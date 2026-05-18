use crate::params::DroneParams;
use crate::state::DroneState;
use nalgebra::{Quaternion, Vector3};

const GRAVITY: f64 = 9.81;

/// Control input - angular velocities of all four engines [rad/s]
/// ordering: [front-right, rear-left, front-left, rear-right]
/// engines 0,3: CW
/// engines 1,2: CCW
#[derive(Debug, Clone, Copy)]
pub struct ControlInput {
    pub motor_speeds: [f64; 4], // [ω0, ω1, ω2, ω3] in rad/s
}

impl ControlInput {
    /// hover: all engines at equal speed
    /// analitycally: 4 * k_thrust * ω² = m * g
    /// => ω = sqrt(m * g / (4 * k_thrust))
    pub fn hover(params: &DroneParams) -> Self {
        let w = (params.mass * GRAVITY / (4.0 * params.k_thrust)).sqrt();

        Self {
            motor_speeds: [w; 4],
        }
    }
}

/// State derivatives - dynamic functions result
/// dstate/dt at given state and input
#[derive(Debug, Clone)]
pub struct StateDot {
    pub velocity: Vector3<f64>,             // ṗ = v
    pub acceleration: Vector3<f64>,         // v̇ = F/m + g
    pub angular_acceleration: Vector3<f64>, // ω̇ = I⁻¹(τ - ω×Iω)
    pub orientation_dot: Quaternion<f64>,   // q̇ = ½ q ⊗ ω
}

/// Main dynamic function
/// no side effects, no global state
/// always same result for same arguments
pub fn derivatives(state: &DroneState, input: &ControlInput, params: &DroneParams) -> StateDot {
    // 1. Forces and torque from engines

    let thrusts = motor_thrusts(input, params);
    let reactive_torques = motor_torques(input, params);
    let f_total: f64 = thrusts.iter().sum();

    // 2. Translation (world frame)

    let thrust_body = Vector3::new(0.0, 0.0, f_total);
    let thrust_world = state.orientation * thrust_body;

    let gravity_world = Vector3::new(0.0, 0.0, -GRAVITY);
    let acceleration = thrust_world / params.mass + gravity_world;

    // 3. Rotation (body frame)

    let l = params.arm_length;
    let w = &state.angular_velocity;

    let tau = Vector3::new(
        l * (thrusts[3] - thrusts[1]), // roll
        l * (thrusts[2] - thrusts[0]), // pitch
        reactive_torques[0] - reactive_torques[1] + reactive_torques[2] - reactive_torques[3], // yaw
    );

    let iw = params.inertia * w;
    let angular_acceleration = params.inertia_inv * (tau - w.cross(&iw));

    // 4. Quaternion derivative

    let omega_quat = Quaternion::from_parts(0.0, *w);
    let orientation_dot = state.orientation.quaternion() * omega_quat * 0.5;

    StateDot {
        velocity: state.velocity,
        acceleration,
        angular_acceleration,
        orientation_dot,
    }
}

/// Helper functions
fn motor_thrusts(input: &ControlInput, params: &DroneParams) -> [f64; 4] {
    input.motor_speeds.map(|w| params.k_thrust * w * w)
}

fn motor_torques(input: &ControlInput, params: &DroneParams) -> [f64; 4] {
    input.motor_speeds.map(|w| params.k_torque * w * w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::DroneParams;
    use crate::state::DroneState;
    use nalgebra::{UnitQuaternion, Vector3};

    fn hovering_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        }
    }

    #[test]
    fn hover_zero_acceleration() {
        let params = DroneParams::mini3();
        let input = ControlInput::hover(&params);
        let state = hovering_state();

        let dot = derivatives(&state, &input, &params);

        assert!(
            dot.acceleration.norm() < 1e-6,
            "Acceleration at hover: {:?}",
            dot.acceleration
        );
    }

    #[test]
    fn no_engines_fall() {
        let params = DroneParams::mini3();
        let input = ControlInput {
            motor_speeds: [0.0; 4],
        };
        let state = hovering_state();

        let dot = derivatives(&state, &input, &params);

        assert!(
            (dot.acceleration.z + 9.81).abs() < 1e-6,
            "az = {}",
            dot.acceleration.z
        );
        assert!(dot.acceleration.x.abs() < 1e-10);
        assert!(dot.acceleration.y.abs() < 1e-10);
    }

    #[test]
    fn bigger_thrust_accelerates_up() {
        let params = DroneParams::mini3();
        let hover_input = ControlInput::hover(&params);

        let w_fast = hover_input.motor_speeds[0] * 1.2;
        let input = ControlInput {
            motor_speeds: [w_fast; 4],
        };
        let state = hovering_state();

        let dot = derivatives(&state, &input, &params);

        assert!(dot.acceleration.z > 0.0, "Drone should accelerate up");
    }
}
