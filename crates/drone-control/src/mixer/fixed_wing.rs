use super::{AttitudeCommand, Mixer};
use drone_model::vehicle::KnownActuatorInput;

pub struct FixedWingMixer {
    cruise_throttle: f64,
}

impl FixedWingMixer {
    pub fn new(cruise_throttle: f64) -> Self {
        Self { cruise_throttle }
    }

    pub fn from_equilibrium(input: KnownActuatorInput) -> Self {
        match input {
            KnownActuatorInput::FixedWing { throttle, .. } => Self {
                cruise_throttle: throttle,
            },
            other => panic!("FixedWingMixer: expected FixedWing input, got {:?}", other),
        }
    }
}

impl Mixer for FixedWingMixer {
    fn mix(&self, cmd: &AttitudeCommand) -> KnownActuatorInput {
        KnownActuatorInput::FixedWing {
            throttle: cmd.throttle.clamp(0.0, 1.0),
            aileron: cmd.roll.clamp(-1.0, 1.0),
            elevator: cmd.pitch.clamp(-1.0, 1.0),
            rudder: cmd.yaw.clamp(-1.0, 1.0),
        }
    }

    fn equilibrium_command(&self) -> AttitudeCommand {
        AttitudeCommand {
            throttle: self.cruise_throttle,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}
