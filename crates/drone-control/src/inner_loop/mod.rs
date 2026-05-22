/// Controller's inner loop - velocity -> control
/// With state, as it has memory
use drone_model::time::TimeStep;

pub mod pid_loop;

pub trait InnerLoop: Send + Sync {
    /// computes control input for given error and time step
    fn compute(&mut self, error: f64, dt: TimeStep) -> f64;

    /// resets internal state
    fn reset(&mut self);
}
