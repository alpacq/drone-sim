/// Velocity profilers - outer loop of the cascade controller position -> velocity
/// Computes target velocity from the position error
/// No internal state - the same output for the same input
pub mod linear;
pub mod sqrt;

/// can be created as static as stateless struct
pub trait VelocityProfiler: Send + Sync {
    /// Computes target velocity for given position error [m] -> [m/s]
    fn compute(&self, error: f64) -> f64;
}
