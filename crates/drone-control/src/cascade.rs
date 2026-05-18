use crate::mixer::{AttitudeCommand, mix};
use crate::pid::Pid;
use drone_model::{dynamics::ControlInput, params::DroneParams, state::DroneState};

// Z axis (altitude) cascade  controller
// roll pitch and yaw set at 0
pub struct AltitudeController {
    pid_position: Pid,
    pid_velocity: Pid,
    hover_throttle: f64,
}

impl AltitudeController {
    pub fn new(params: &DroneParams) -> Self {
        let hover_throttle = 0.5;

        Self {
            // position loop - slow, not aggressive
            pid_position: Pid::new(1.0, 0.0, 0.2, 2.0, 3.0),

            // velocity loop - fast, more aggressive
            pid_velocity: Pid::new(0.3, 0.05, 0.1, 0.3, 0.5),
            hover_throttle,
        }
    }

    pub fn update(&mut self, state: &DroneState, target_z: f64, dt: f64) -> ControlInput {
        // outer loop: position -> given velocity
        let error_z = target_z - state.position.z;
        let target_vz = self.pid_position.update(error_z, dt);

        // inner loop: velocity -> throttle
        let error_vz = target_vz - state.velocity.z;
        let throttle_delta = self.pid_velocity.update(error_vz, dt);

        let throttle = (self.hover_throttle + throttle_delta).clamp(0.0, 1.0);

        let cmd = AttitudeCommand {
            throttle,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };

        mix(&cmd, self.max_motor_speed())
    }

    pub fn reset(&mut self) {
        self.pid_position.reset();
        self.pid_velocity.reset();
    }

    fn max_motor_speed(&self) -> f64 {
        1000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{params::DroneParams, state::DroneState};
    use nalgebra::{UnitQuaternion, Vector3};

    fn state_on_ground() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        }
    }

    #[test]
    fn controller_gives_more_throttle_if_too_low() {
        let params = DroneParams::mini3();
        let mut ctrl = AltitudeController::new(&params);
        let state = state_on_ground();

        // Cel: 5m, jesteśmy na 0m → kontroler powinien dać > hover throttle
        let input = ctrl.update(&state, 5.0, 0.01);
        let avg_speed: f64 = input.motor_speeds.iter().sum::<f64>() / 4.0;

        // Hover speed to ~500 (0.5 * 1000)
        assert!(
            avg_speed > 500.0,
            "With target above engines should rotate faster than hover"
        );
    }

    #[test]
    fn controler_gives_less_throttle_if_too_high() {
        let params = DroneParams::mini3();
        let mut ctrl = AltitudeController::new(&params);

        // Jesteśmy na 10m, cel to 5m
        let state = DroneState {
            position: Vector3::new(0.0, 0.0, 10.0),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        };

        let input = ctrl.update(&state, 5.0, 0.01);
        let avg_speed: f64 = input.motor_speeds.iter().sum::<f64>() / 4.0;

        assert!(
            avg_speed < 500.0,
            "With target below engines should rotate slower than hover"
        );
    }
}
