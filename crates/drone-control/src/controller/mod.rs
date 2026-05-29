use crate::target::FlightTarget;
use drone_model::{state::DroneState, time::TimeStep, vehicle::KnownActuatorInput};

pub trait Controller: Send + Sync {
    fn update(
        &mut self,
        state: &DroneState,
        target: &FlightTarget,
        dt: TimeStep,
    ) -> KnownActuatorInput;

    fn reset(&mut self);

    fn name(&self) -> &str;

    /// Return the most recently computed planned z-trajectory over the
    /// prediction horizon.  Index 0 is the current z, index k is the
    /// predicted z after k MPC prediction steps.
    ///
    /// Only implemented by [`MpcController`]; all other controllers
    /// return `None`.
    fn planned_z_horizon(&self) -> Option<&[f64]> {
        None
    }
}
