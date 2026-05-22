/// Translates high level attitude commands into specific control output
use drone_model::vehicle::KnownActuatorInput;

pub mod fixed_wing;
pub mod quadrotor;

#[derive(Debug, Clone, Copy)]
pub struct AttitudeCommand {
    pub throttle: f64, // [0,1]
    pub roll: f64,     // [-1,1]
    pub pitch: f64,    // [-1,1]
    pub yaw: f64,      // [-1,1]
}

pub trait Mixer: Send + Sync {
    fn mix(&self, cmd: &AttitudeCommand) -> KnownActuatorInput;
    fn equilibrium_command(&self) -> AttitudeCommand;
}
