use super::InnerLoop;
use crate::pid::Pid;
use drone_model::time::TimeStep;

pub struct PidLoop(pub Pid);

impl PidLoop {
    pub fn new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self {
        Self(Pid::new(kp, ki, kd, integral_limit, output_limit))
    }
}

impl InnerLoop for PidLoop {
    fn compute(&mut self, error: f64, dt: TimeStep) -> f64 {
        self.0.update(error, dt)
    }

    fn reset(&mut self) {
        self.0.reset();
    }
}
