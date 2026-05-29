//! Time-varying trajectory planning for open-loop path tracking.
//!
//! The runner calls [`Trajectory::target`] every simulation step to get a
//! [`FlightTarget`] that varies with time.  Three built-in trajectories are
//! provided: [`HoldTrajectory`], [`WaypointTrajectory`], and
//! [`CircleTrajectory`].

use crate::target::FlightTarget;

/// Returns a time-varying flight target for open-loop trajectory tracking.
///
/// Implement this for any planned path; the runner calls `target(time_s)`
/// every step.
pub trait Trajectory: Send + Sync {
    /// Compute the desired flight target at the given simulation time.
    fn target(&self, time_s: f64) -> FlightTarget;
}

/// Always returns the same target — useful as a no-op wrapper.
#[derive(Debug, Clone)]
pub struct HoldTrajectory {
    /// The constant target returned for all time values.
    pub inner: FlightTarget,
}

impl Trajectory for HoldTrajectory {
    fn target(&self, _time_s: f64) -> FlightTarget {
        self.inner.clone()
    }
}

/// Piecewise-linear trajectory through timed waypoints.
///
/// Format: `Vec<(time_s, FlightTarget)>` sorted ascending by time.
/// - Before the first waypoint → first waypoint held.
/// - After the last waypoint → last waypoint held.
/// - Between waypoints → linearly interpolate the `Some`-axes only.
#[derive(Debug, Clone)]
pub struct WaypointTrajectory {
    /// Sorted `(time_s, FlightTarget)` pairs — must have at least one entry.
    waypoints: Vec<(f64, FlightTarget)>,
}

impl WaypointTrajectory {
    /// Create a new waypoint trajectory from a list of `(time_s, FlightTarget)` pairs.
    ///
    /// # Panics
    /// Panics if `wps` is empty.
    pub fn new(mut wps: Vec<(f64, FlightTarget)>) -> Self {
        assert!(
            !wps.is_empty(),
            "WaypointTrajectory needs at least one waypoint"
        );
        wps.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        Self { waypoints: wps }
    }
}

/// Linearly interpolate two `Option<f64>` values.
///
/// - Both `Some` → lerp.
/// - Only one `Some` → hold that value.
/// - Both `None` → `None`.
fn lerp_axis(a: Option<f64>, b: Option<f64>, alpha: f64) -> Option<f64> {
    match (a, b) {
        (Some(va), Some(vb)) => Some(va + alpha * (vb - va)),
        (Some(v), None) | (None, Some(v)) => Some(v),
        (None, None) => None,
    }
}

impl Trajectory for WaypointTrajectory {
    fn target(&self, time_s: f64) -> FlightTarget {
        // Before first waypoint → hold first.
        if time_s <= self.waypoints[0].0 {
            return self.waypoints[0].1.clone();
        }
        // After last waypoint → hold last.
        let last = self.waypoints.len() - 1;
        if time_s >= self.waypoints[last].0 {
            return self.waypoints[last].1.clone();
        }
        // Find the two bounding waypoints.
        let idx = self
            .waypoints
            .partition_point(|&(t, _)| t <= time_s)
            .saturating_sub(1);
        let (t0, ref a) = self.waypoints[idx];
        let (t1, ref b) = self.waypoints[idx + 1];
        let alpha = (time_s - t0) / (t1 - t0);

        FlightTarget {
            x: lerp_axis(a.x, b.x, alpha),
            y: lerp_axis(a.y, b.y, alpha),
            z: lerp_axis(a.z, b.z, alpha),
            yaw: lerp_axis(a.yaw, b.yaw, alpha),
        }
    }
}

/// Horizontal circular orbit at a fixed altitude.
#[derive(Debug, Clone)]
pub struct CircleTrajectory {
    /// Orbit centre X [m].
    pub cx: f64,
    /// Orbit centre Y [m].
    pub cy: f64,
    /// Orbit radius [m].
    pub radius: f64,
    /// Angular velocity [rad/s]; positive = CCW.
    pub omega: f64,
    /// Constant altitude [m].
    pub altitude_m: f64,
}

impl Trajectory for CircleTrajectory {
    fn target(&self, time_s: f64) -> FlightTarget {
        let angle = self.omega * time_s;
        FlightTarget::position(
            self.cx + self.radius * angle.cos(),
            self.cy + self.radius * angle.sin(),
            self.altitude_m,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hold_stays_constant() {
        let traj = HoldTrajectory {
            inner: FlightTarget::altitude(5.0),
        };
        let t0 = traj.target(0.0);
        let t999 = traj.target(999.0);
        assert_eq!(t0.z, Some(5.0));
        assert_eq!(t999.z, Some(5.0));
        assert_eq!(t0.x, None);
        assert_eq!(t999.x, None);
    }

    #[test]
    fn waypoint_interpolates_midpoint() {
        let wps = vec![
            (0.0, FlightTarget::altitude(0.0)),
            (10.0, FlightTarget::altitude(10.0)),
        ];
        let traj = WaypointTrajectory::new(wps);
        let mid = traj.target(5.0);
        assert!(
            (mid.z.unwrap() - 5.0).abs() < 1e-10,
            "z at midpoint = {:?}",
            mid.z
        );
    }

    #[test]
    fn waypoint_holds_before_first() {
        let wps = vec![
            (2.0, FlightTarget::altitude(3.0)),
            (10.0, FlightTarget::altitude(10.0)),
        ];
        let traj = WaypointTrajectory::new(wps);
        let early = traj.target(0.0);
        assert_eq!(early.z, Some(3.0));
    }

    #[test]
    fn waypoint_holds_after_last() {
        let wps = vec![
            (0.0, FlightTarget::altitude(0.0)),
            (5.0, FlightTarget::altitude(8.0)),
        ];
        let traj = WaypointTrajectory::new(wps);
        let late = traj.target(100.0);
        assert_eq!(late.z, Some(8.0));
    }

    #[test]
    fn waypoint_partial_axes() {
        // First waypoint has x=Some, second has x=None → hold first x value
        let a = FlightTarget {
            x: Some(1.0),
            y: None,
            z: Some(0.0),
            yaw: None,
        };
        let b = FlightTarget {
            x: None,
            y: Some(5.0),
            z: Some(10.0),
            yaw: None,
        };
        let traj = WaypointTrajectory::new(vec![(0.0, a), (10.0, b)]);
        let mid = traj.target(5.0);
        assert_eq!(mid.x, Some(1.0)); // held from a
        assert_eq!(mid.y, Some(5.0)); // held from b
        assert!((mid.z.unwrap() - 5.0).abs() < 1e-10); // interpolated
    }

    #[test]
    fn circle_stays_on_orbit() {
        let traj = CircleTrajectory {
            cx: 1.0,
            cy: 2.0,
            radius: 3.0,
            omega: 0.5,
            altitude_m: 10.0,
        };
        for i in 0..10 {
            let t = i as f64 * 1.3;
            let ft = traj.target(t);
            let dx = ft.x.unwrap() - 1.0;
            let dy = ft.y.unwrap() - 2.0;
            let dist = (dx * dx + dy * dy).sqrt();
            assert!(
                (dist - 3.0).abs() < 1e-10,
                "t={t}: dist from centre = {dist}"
            );
            assert_eq!(ft.z, Some(10.0));
        }
    }
}
