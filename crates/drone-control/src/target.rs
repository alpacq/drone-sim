/// Desired flight state expressed as per-axis optional setpoints.
///
/// `None` on an axis means "do not control this axis".  The cascade controller
/// and LQI integrators respect this: a `None` axis produces zero error and zero
/// integral accumulation, so the vehicle simply stabilises at whatever position
/// it is currently at on that axis.
///
/// # Why per-axis rather than a single `Option<Vector3>`?
///
/// With a flat `Vector3` there is no way to say "hold altitude only" without
/// implicitly commanding X=0 and Y=0.  The previous `altitude(z)` helper did
/// exactly that, which would pull any drone that started away from the origin
/// back to (0, 0, z) instead of holding its current horizontal position.
#[derive(Debug, Clone)]
pub struct FlightTarget {
    /// Desired X position [m].  `None` = do not control X.
    pub x: Option<f64>,
    /// Desired Y position [m].  `None` = do not control Y.
    pub y: Option<f64>,
    /// Desired Z (altitude) [m].  `None` = do not control altitude.
    pub z: Option<f64>,
    /// Desired yaw angle [rad].  `None` = do not control yaw.
    pub yaw: Option<f64>,
}

impl FlightTarget {
    /// Altitude-only target: only Z is controlled, X and Y are left free.
    pub fn altitude(z: f64) -> Self {
        Self { x: None, y: None, z: Some(z), yaw: None }
    }

    /// Full 3-D position target (no yaw command).
    pub fn position(x: f64, y: f64, z: f64) -> Self {
        Self { x: Some(x), y: Some(y), z: Some(z), yaw: None }
    }

    /// Full 3-D position + yaw target.
    pub fn full(x: f64, y: f64, z: f64, yaw: f64) -> Self {
        Self { x: Some(x), y: Some(y), z: Some(z), yaw: Some(yaw) }
    }
}
