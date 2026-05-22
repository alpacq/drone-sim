/// Linear velocity profiler
/// v = Kp × error
use super::VelocityProfiler;

pub struct LinearProfiler {
    pub kp: f64,
    pub v_max: f64,
}

impl LinearProfiler {
    pub fn new(kp: f64, v_max: f64) -> Self {
        Self { kp, v_max }
    }
}

impl VelocityProfiler for LinearProfiler {
    fn compute(&self, error: f64) -> f64 {
        (self.kp * error).clamp(-self.v_max, self.v_max)
    }
}
