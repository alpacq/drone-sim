use drone_model::dynamics::ControlInput;

#[derive(Debug, Clone, Copy)]
pub struct AttitudeCommand {
    pub throttle: f64, // [-1,1]
    pub roll: f64,     // [-1,1]
    pub pitch: f64,    // [-1,1]
    pub yaw: f64,      // [-1,1]
}

// Changes high level command to engine speeds
// X-frame geometry (top view):
//   2(CCW)  1(CW)
//      \   /
//       [B]     ← nose up (+x)
//      /   \
//   3(CW)  4(CCW)
pub fn mix(cmd: &AttitudeCommand, max_rpm: f64) -> ControlInput {
    let base = cmd.throttle * max_rpm;

    let w0 = base - cmd.pitch - cmd.roll + cmd.yaw; // front-right, CW
    let w1 = base - cmd.pitch + cmd.roll - cmd.yaw; // front-left, CCW
    let w2 = base + cmd.pitch + cmd.roll + cmd.yaw; // rear-left,  CW
    let w3 = base + cmd.pitch - cmd.roll - cmd.yaw; // rear-right, CCW

    ControlInput {
        motor_speeds: [w0.max(0.0), w1.max(0.0), w2.max(0.0), w3.max(0.0)],
    }
}
