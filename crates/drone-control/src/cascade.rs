use crate::mixer::{AttitudeCommand, mix};
use crate::pid::Pid;
use drone_model::{dynamics::ControlInput, params::DroneParams, state::DroneState, time::TimeStep};

// Max approach speed [m/s]
const V_MAX: f64 = 1.0;
// Deceleration used to shape the velocity profile [m/s²]
// target_vz = sign(err) * min(sqrt(2 * BRAKE_ACCEL * |err|), V_MAX)
// This guarantees the drone slows down naturally before the setpoint.
const BRAKE_ACCEL: f64 = 1.5;

// Z axis (altitude) cascade controller
// Outer loop: sqrt velocity profiler (replaces linear position PID)
// Inner loop: velocity PI — NO D term (D on velocity causes limit-cycling
//   because dω/dt from thrust changes is ~20 m/s², which with any kd > 0.02
//   immediately saturates the output every step)
pub struct AltitudeController {
    pid_velocity: Pid,
    hover_motor_speed: f64,
    max_motor_speed: f64,
}

impl AltitudeController {
    pub fn new(params: &DroneParams) -> Self {
        // ω = √(m*g / (4*k_thrust))
        let hover_motor_speed = (params.mass * 9.81 / (4.0 * params.k_thrust)).sqrt();
        let max_motor_speed = hover_motor_speed * 1.77152318;

        Self {
            // kd=0: derivative on velocity causes limit-cycling (see module comment)
            pid_velocity: Pid::new(0.3, 0.1, 0.0, 0.45, 0.45),
            hover_motor_speed,
            max_motor_speed,
        }
    }

    pub fn update(&mut self, state: &DroneState, target_z: f64, dt: TimeStep) -> ControlInput {
        // Outer loop: sqrt velocity profile
        // Velocity setpoint scales with sqrt(|error|) so the drone decelerates
        // smoothly rather than arriving at full speed and overshooting.
        let error_z = target_z - state.position.z;
        let target_vz = error_z.signum() * (2.0 * BRAKE_ACCEL * error_z.abs()).sqrt().min(V_MAX);

        // Inner loop: velocity PI
        let error_vz = target_vz - state.velocity.z;
        let throttle_delta = self.pid_velocity.update(error_vz, dt);

        let hover_throttle = self.hover_motor_speed / self.max_motor_speed;
        let throttle = (hover_throttle + throttle_delta).clamp(0.0, 1.0);

        let cmd = AttitudeCommand {
            throttle,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };

        mix(&cmd, self.max_motor_speed)
    }

    pub fn reset(&mut self) {
        self.pid_velocity.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{params::DroneParams, state::DroneState, time::TimeStep};
    use nalgebra::{UnitQuaternion, Vector3};

    fn dt() -> TimeStep {
        TimeStep::constant(0.01)
    }

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
        let hover_speed = (params.mass * 9.81 / (4.0 * params.k_thrust)).sqrt();

        // Target: 5m, we're at 0m
        let input = ctrl.update(&state, 5.0, dt());
        let avg_speed: f64 = input.motor_speeds.sum() / 4.0;

        // avg speed must be above hover speed (~632.5 rad/s for Mini 3)
        assert!(
            avg_speed > hover_speed,
            "With target above, engines should rotate faster than hover ({:.1} rad/s), got {:.1}",
            hover_speed,
            avg_speed
        );
    }

    #[test]
    fn controler_gives_less_throttle_if_too_high() {
        let params = DroneParams::mini3();
        let mut ctrl = AltitudeController::new(&params);
        let hover_speed = (params.mass * 9.81 / (4.0 * params.k_thrust)).sqrt();

        // We're at 10m, target is 5m
        let state = DroneState {
            position: Vector3::new(0.0, 0.0, 10.0),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        };

        let input = ctrl.update(&state, 5.0, dt());
        let avg_speed: f64 = input.motor_speeds.sum() / 4.0;

        // avg speed must be below hover speed (~632.5 rad/s for Mini 3)
        assert!(
            avg_speed < hover_speed,
            "With target below, engines should rotate slower than hover ({:.1} rad/s), got {:.1}",
            hover_speed,
            avg_speed
        );
    }

    #[test]
    fn diagnostics_hover_speed() {
        let params = DroneParams::mini3();
        let ctrl = AltitudeController::new(&params);

        // Sprawdź czy hover_motor_speed zgadza się z dynamics
        let hover_input = ControlInput::hover(&params);
        let hover_speed_dynamics = hover_input.motor_speeds.sum() / 4.0;

        // Muszą być równe — inaczej kontroler i dynamika używają różnych skal
        assert!(
            (ctrl.hover_motor_speed - hover_speed_dynamics).abs() < 0.01,
            "Difference: controller={:.2} dynamics={:.2}",
            ctrl.hover_motor_speed,
            hover_speed_dynamics
        );
    }
}
