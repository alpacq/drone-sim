use drone_model::{
    dynamics::{ControlInput, StateDot, derivatives},
    params::DroneParams,
    state::DroneState,
};
use nalgebra::UnitQuaternion;

// -- Trait -----------------------------------------------------------------------------------------------
//
// Integrating method for movement equations of the drone
// Trait is objectable ('dyn Integrator') - we can have different implementations with the same interface
pub trait Integrator {
    fn step(
        &self,
        state: &DroneState,
        input: &ControlInput,
        params: &DroneParams,
        dt: f64,
    ) -> DroneState;
}

// Common function apply_dot
// Applies derivatives to state
// Normalizes quaternion after every step to avoid numerical drift
pub fn apply_dot(state: &DroneState, dot: &StateDot, dt: f64) -> DroneState {
    // position and velocity: vector sum
    let position = state.position + dot.velocity * dt;
    let velocity = state.velocity + dot.acceleration * dt;

    // angular velocity: vector sum
    let angular_velocity = state.angular_velocity + dot.angular_acceleration * dt;

    // quaternion: adding derivative and normalizing
    let q_raw = state.orientation.quaternion() + dot.orientation_dot * dt;
    let orientation = UnitQuaternion::from_quaternion(q_raw);

    DroneState {
        position,
        velocity,
        angular_velocity,
        orientation,
    }
}

// -- Euler ------------------------------------------------------------------------------------------------
//
// Euler method - O(dt) accuracy
// Useful for comparing with RK4 and testing stability
pub struct Euler;

impl Integrator for Euler {
    fn step(
        &self,
        state: &DroneState,
        input: &ControlInput,
        params: &DroneParams,
        dt: f64,
    ) -> DroneState {
        let dot = derivatives(state, input, params);
        apply_dot(state, &dot, dt)
    }
}

// -- RK4 --------------------------------------------------------------------------------------------------
//
// Runge-Kutta 4th order method - O(dt⁴) accuracy
// standard choice for flight simulators
pub struct RK4;

impl Integrator for RK4 {
    fn step(
        &self,
        state: &DroneState,
        input: &ControlInput,
        params: &DroneParams,
        dt: f64,
    ) -> DroneState {
        let k1 = derivatives(state, input, params);

        let state_k2 = apply_dot(state, &k1, dt / 2.0);
        let k2 = derivatives(&state_k2, input, params);

        let state_k3 = apply_dot(state, &k2, dt / 2.0);
        let k3 = derivatives(&state_k3, input, params);

        let state_k4 = apply_dot(state, &k3, dt);
        let k4 = derivatives(&state_k4, input, params);

        let dot_combined = weighted_average(&k1, &k2, &k3, &k4);
        apply_dot(state, &dot_combined, dt)
    }
}

fn weighted_average(k1: &StateDot, k2: &StateDot, k3: &StateDot, k4: &StateDot) -> StateDot {
    StateDot {
        velocity: (k1.velocity + k2.velocity * 2.0 + k3.velocity * 2.0 + k4.velocity) / 6.0,
        acceleration: (k1.acceleration
            + k2.acceleration * 2.0
            + k3.acceleration * 2.0
            + k4.acceleration)
            / 6.0,
        angular_acceleration: (k1.angular_acceleration
            + k2.angular_acceleration * 2.0
            + k3.angular_acceleration * 2.0
            + k4.angular_acceleration)
            / 6.0,
        orientation_dot: (k1.orientation_dot
            + k2.orientation_dot * 2.0
            + k3.orientation_dot * 2.0
            + k4.orientation_dot)
            / 6.0,
    }
}
