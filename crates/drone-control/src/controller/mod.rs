use crate::target::FlightTarget;
use drone_model::{state::DroneState, time::TimeStep, vehicle::KnownActuatorInput};

pub mod cascade;

pub trait Controller: Send + Sync {
    fn update(
        &mut self,
        state: &DroneState,
        target: &FlightTarget,
        dt: TimeStep,
    ) -> KnownActuatorInput;

    fn reset(&mut self);

    fn name(&self) -> &str;
}
