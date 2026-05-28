use drone_model::{
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, StateDot, VehicleModel},
};
use nalgebra::UnitQuaternion;

// -- Trait -----------------------------------------------------------------------------------------------
//
// Integrating method for movement equations of the drone
// Trait is objectable ('dyn Integrator') - we can have different implementations with the same interface
pub trait Integrator: Send + Sync {
    fn step(
        &self,
        model: &dyn VehicleModel,
        state: &DroneState,
        input: &KnownActuatorInput,
        dt: TimeStep,
    ) -> DroneState;
}

// Common function apply_dot
// Applies derivatives to state
// Normalizes quaternion after every step to avoid numerical drift
pub fn apply_dot(state: &DroneState, dot: &StateDot, dt: TimeStep) -> DroneState {
    let dt = dt.seconds();
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
        actuator_state: state.actuator_state.clone(),
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
        model: &dyn VehicleModel,
        state: &DroneState,
        input: &KnownActuatorInput,
        dt: TimeStep,
    ) -> DroneState {
        let dot = model.derivatives(state, input);
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
        model: &dyn VehicleModel,
        state: &DroneState,
        input: &KnownActuatorInput,
        dt: TimeStep,
    ) -> DroneState {
        let k1 = model.derivatives(state, input);

        let state_k2 = apply_dot(state, &k1, dt.half());
        let k2 = model.derivatives(&state_k2, input);

        let state_k3 = apply_dot(state, &k2, dt.half());
        let k3 = model.derivatives(&state_k3, input);

        let state_k4 = apply_dot(state, &k3, dt);
        let k4 = model.derivatives(&state_k4, input);

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

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{
        motor::MotorArray,
        state::DroneState,
        time::TimeStep,
        vehicle::{KnownActuatorInput, quadrotor::QuadrotorModel},
    };
    use nalgebra::{UnitQuaternion, Vector3};

    fn ground_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    /// Helper: creates input hover for quadrotor
    fn hover_input(model: &QuadrotorModel) -> KnownActuatorInput {
        model.equilibrium_input()
    }

    #[test]
    fn euler_hover_doesnt_fall() {
        let model = QuadrotorModel::mini3();
        let dt = TimeStep::constant(0.005);
        let mut state = ground_state();

        for _ in 0..200 {
            state = Euler.step(&model, &state, &hover_input(&model), dt);
        }

        // After 1s hover Euler should be close to z=0
        assert!(
            state.position.z.abs() < 0.1,
            "Euler hover: z = {:.4}",
            state.position.z
        );
    }

    #[test]
    fn rk4_hover_doesnt_fall() {
        let model = QuadrotorModel::mini3();
        let dt = TimeStep::constant(0.005);
        let mut state = ground_state();

        for _ in 0..200 {
            state = RK4.step(&model, &state, &hover_input(&model), dt);
        }

        assert!(
            state.position.z.abs() < 0.01,
            "RK4 hover: z = {:.4}",
            state.position.z
        );
    }

    #[test]
    fn without_motors_drone_falls() {
        let model = QuadrotorModel::mini3();
        let dt = TimeStep::constant(0.005);
        let input = KnownActuatorInput::Quadrotor(MotorArray::uniform(0.0));
        let mut state = ground_state();

        for _ in 0..200 {
            state = RK4.step(&model, &state, &input, dt);
        }

        // The model uses quadratic drag: F = k_drag * v²
        // Terminal velocity: v_t = sqrt(m*g / k_drag) ≈ 4.0 m/s for Mini 3
        // Analytical z(t) for quadratic drag: -(v_t²/g) * ln(cosh(g*t/v_t))
        // → after 1s: z ≈ -2.9 m (less than free-fall -4.9 m due to drag)
        let m = model.params.mass;
        let g = 9.80665_f64;
        let k = model.params.k_drag;
        let v_t = (m * g / k).sqrt();
        let expected = -(v_t * v_t / g) * (g / v_t).cosh().ln();
        assert!(
            (state.position.z - expected).abs() < 0.15,
            "Expected z ≈ {:.2} (quadratic drag), got {:.2}",
            expected,
            state.position.z
        );
    }

    #[test]
    fn rk4_more_accurate_than_euler() {
        let model = QuadrotorModel::mini3();
        let big_dt = TimeStep::constant(0.05);
        let ref_dt = TimeStep::constant(0.0001);

        // Input: 20% above hover
        let boosted_input = |m: &QuadrotorModel| {
            let eq = m.equilibrium_input();
            match eq {
                KnownActuatorInput::Quadrotor(s) => {
                    KnownActuatorInput::Quadrotor(s.map(|w| w * 1.2))
                }
                other => other,
            }
        };

        // Reference with very small dt and RK4
        let mut ref_state = ground_state();
        let steps_ref = (1.0 / ref_dt.seconds()) as usize;
        for _ in 0..steps_ref {
            ref_state = RK4.step(&model, &ref_state, &boosted_input(&model), ref_dt);
        }
        let z_ref = ref_state.position.z;

        // Euler with big dt
        let mut euler_state = ground_state();
        let steps_big = (1.0 / big_dt.seconds()) as usize;
        for _ in 0..steps_big {
            euler_state = Euler.step(&model, &euler_state, &boosted_input(&model), big_dt);
        }

        // RK4 with big dt
        let mut rk4_state = ground_state();
        for _ in 0..steps_big {
            rk4_state = RK4.step(&model, &rk4_state, &boosted_input(&model), big_dt);
        }

        let err_euler = (euler_state.position.z - z_ref).abs();
        let err_rk4 = (rk4_state.position.z - z_ref).abs();

        assert!(
            err_rk4 < err_euler,
            "RK4 (error={:.4}m) should be more accurate than Euler (error={:.4}m)",
            err_rk4,
            err_euler
        );
    }

    #[test]
    fn fixed_wing_doesnt_panic() {
        use drone_model::vehicle::fixed_wing::F16Model;

        let model = F16Model::f16a();
        let state = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::new(15.0, 0.0, 0.0),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };
        let input = model.equilibrium_input();
        let dt = TimeStep::constant(0.005);

        let next = RK4.step(&model, &state, &input, dt);
        assert!(next.position.x.is_finite());
        assert!(next.position.z.is_finite());
    }
}
