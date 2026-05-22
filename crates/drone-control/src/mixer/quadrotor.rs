use super::{AttitudeCommand, Mixer};
use drone_model::{motor::MotorArray, vehicle::KnownActuatorInput};

/// Mixer for X-frame quadrotor vehicle
pub struct QuadrotorMixer {
    hover_motor_speed: f64,
    max_motor_speed: f64,
}

impl QuadrotorMixer {
    pub fn new(hover_motor_speed: f64, max_motor_speed: f64) -> Self {
        Self {
            hover_motor_speed,
            max_motor_speed,
        }
    }

    pub fn from_equilibrium(input: KnownActuatorInput) -> Self {
        match input {
            KnownActuatorInput::Quadrotor(speeds) => {
                let hover = speeds.sum() / 4.0;
                Self {
                    hover_motor_speed: hover,
                    max_motor_speed: hover * 1.77152318,
                }
            }
            other => panic!("QuadrotorMixer: expected Quadrotor input, got {:?}", other),
        }
    }
}

impl Mixer for QuadrotorMixer {
    /// Changes high level command to engine speeds
    /// X-frame geometry (top view):
    ///   1(CCW)  0(CW)
    ///      \   /
    ///       [B]     ← nose up (+x)
    ///      /   \
    ///   2(CW)  3(CCW)
    fn mix(&self, cmd: &AttitudeCommand) -> KnownActuatorInput {
        let base = cmd.throttle * self.max_motor_speed;
        let r = cmd.roll * self.max_motor_speed * 0.5;
        let p = cmd.pitch * self.max_motor_speed * 0.5;
        let y = cmd.yaw * self.max_motor_speed * 0.5;

        KnownActuatorInput::Quadrotor(MotorArray::new(
            (base - p - r + y).max(0.0), // FrontRight, CW
            (base - p + r - y).max(0.0), // FrontLeft,  CCW
            (base + p + r + y).max(0.0), // RearLeft,   CW
            (base + p - r - y).max(0.0), // RearRight,  CCW
        ))
    }

    fn equilibrium_command(&self) -> AttitudeCommand {
        AttitudeCommand {
            throttle: self.hover_motor_speed / self.max_motor_speed,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}
