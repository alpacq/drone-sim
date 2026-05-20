use drone_model::{dynamics::ControlInput, motor::MotorArray};

#[derive(Debug, Clone, Copy)]
pub struct AttitudeCommand {
    pub throttle: f64, // [-1,1]
    pub roll: f64,     // [-1,1]
    pub pitch: f64,    // [-1,1]
    pub yaw: f64,      // [-1,1]
}

// Changes high level command to engine speeds
// X-frame geometry (top view):
///   1(CCW)  0(CW)
///      \   /
///       [B]     ← nose up (+x)
///      /   \
///   2(CW)  3(CCW)
pub fn mix(cmd: &AttitudeCommand, max_speed: f64) -> ControlInput {
    let base = cmd.throttle * max_speed;
    let r = cmd.roll * max_speed * 0.5;
    let p = cmd.pitch * max_speed * 0.5;
    let y = cmd.yaw * max_speed * 0.5;

    let speeds = MotorArray::new(
        (base - p - r + y).max(0.0), // FrontRight, CW
        (base - p + r - y).max(0.0), // FrontLeft,  CCW
        (base + p + r + y).max(0.0), // RearLeft,   CW
        (base + p - r - y).max(0.0), // RearRight,  CCW
    );

    ControlInput {
        motor_speeds: speeds,
    }
}
