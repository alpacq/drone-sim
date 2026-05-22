/// Sqrt velocity profiler - kinematic slowdown profile
/// Target velocity: v = sign(e) × min(√(2·a·|e|), v_max)
use super::VelocityProfiler;

pub struct SqrtProfiler {
    /// Brake acceleration [m/s²]
    pub brake_accel: f64,
    /// Max approach velocity [m/s]
    pub v_max: f64,
}

impl SqrtProfiler {
    pub fn new(brake_accel: f64, v_max: f64) -> Self {
        Self { brake_accel, v_max }
    }

    /// for Z axis
    pub fn for_altitude() -> Self {
        Self::new(1.5, 1.0)
    }

    /// for XY plane
    pub fn for_horizontal() -> Self {
        Self::new(2.0, 3.0)
    }
}

impl VelocityProfiler for SqrtProfiler {
    fn compute(&self, error: f64) -> f64 {
        error.signum()
            * (2.0 * self.brake_accel * error.abs())
                .sqrt()
                .min(self.v_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_error_zero_velocity() {
        let p = SqrtProfiler::for_altitude();
        assert_eq!(p.compute(0.0), 0.0);
    }

    #[test]
    fn positive_error_positive_velocity() {
        let p = SqrtProfiler::for_altitude();
        assert!(p.compute(2.0) > 0.0);
    }

    #[test]
    fn negative_error_negative_velocity() {
        let p = SqrtProfiler::for_altitude();
        assert!(p.compute(-2.0) < 0.0);
    }

    #[test]
    fn velocity_limited_by_v_max() {
        let p = SqrtProfiler::for_altitude();
        // Bardzo duży błąd — prędkość nie może przekroczyć v_max
        assert!(p.compute(1000.0) <= p.v_max + 1e-10);
    }

    #[test]
    fn symmetry() {
        let p = SqrtProfiler::for_altitude();
        assert!((p.compute(5.0) + p.compute(-5.0)).abs() < 1e-10);
    }
}
