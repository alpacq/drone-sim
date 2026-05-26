use crate::scenario::MetricKind;
use drone_control::target::FlightTarget;
use drone_sim::runner::SimFrame;

pub fn compute(metric: &MetricKind, frames: &[SimFrame], target: &FlightTarget) -> f64 {
    match metric {
        MetricKind::PositionRmsZ => position_rms_z(frames, target),
        MetricKind::PositionMaxErrorZ => position_max_error_z(frames, target),
        MetricKind::OvershootPercent => overshoot_percent(frames, target),
        MetricKind::SettlingTimeS => settling_time_s(frames, target),
    }
}

/// Root Mean Square of z position error
/// RMS = √( (1/N) × Σ(z_i - z_target)² )
fn position_rms_z(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    if frames.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = frames
        .iter()
        .map(|f| (f.state.position.z - target.position.unwrap_or_default().z).powi(2))
        .sum();
    (sum_sq / frames.len() as f64).sqrt()
}

/// Maximum Z position error during whole flight
fn position_max_error_z(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    frames
        .iter()
        .map(|f| (f.state.position.z - target.position.unwrap_or_default().z).abs())
        .fold(0.0_f64, f64::max)
}

/// overshoot - how much percent drone overreached target
fn overshoot_percent(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    let initial_z = frames.first().map(|f| f.state.position.z).unwrap_or(0.0);

    if initial_z >= target.position.unwrap_or_default().z {
        return 0.0;
    }

    let max_z = frames
        .iter()
        .map(|f| f.state.position.z)
        .fold(f64::NEG_INFINITY, f64::max);

    if max_z <= target.position.unwrap_or_default().z {
        return 0.0;
    }

    let overshoot = max_z - target.position.unwrap_or_default().z;
    let total_range = target.position.unwrap_or_default().z - initial_z;
    (overshoot / total_range) * 100.0
}

/// settling time - how many seconds until error z < 0.1m stable
fn settling_time_s(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    let threshold = 0.1;
    let last_violation = frames.iter().enumerate().rev().find(|(_, f)| {
        (f.state.position.z - target.position.unwrap_or_default().z).abs() >= threshold
    });

    match last_violation {
        Some((idx, _)) => frames[idx].time,
        None => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::state::DroneState;
    use drone_sim::runner::SimFrame;
    use nalgebra::{UnitQuaternion, Vector3};

    fn frame(time: f64, z: f64) -> SimFrame {
        SimFrame {
            time,
            state: DroneState {
                position: Vector3::new(0.0, 0.0, z),
                velocity: Vector3::zeros(),
                orientation: UnitQuaternion::identity(),
                angular_velocity: Vector3::zeros(),
                actuator_state: None,
            },
        }
    }

    #[test]
    fn rms_for_constant_error() {
        // Zawsze 1m od celu → RMS = 1.0
        let frames: Vec<_> = (0..100).map(|i| frame(i as f64 * 0.01, 4.0)).collect();
        let rms = position_rms_z(&frames, &FlightTarget::altitude(5.0));
        assert!((rms - 1.0).abs() < 1e-10);
    }

    #[test]
    fn no_overshoot_when_never_reaches_target() {
        let frames: Vec<_> = (0..100)
            .map(|i| frame(i as f64 * 0.01, i as f64 * 0.04))
            .collect();
        // Max z = 3.96m, cel = 5.0m → brak overshootu
        let overshoot = overshoot_percent(&frames, &FlightTarget::altitude(5.0));
        assert_eq!(overshoot, 0.0);
    }

    #[test]
    fn settling_time_when_always_in_threshold() {
        let frames: Vec<_> = (0..100).map(|i| frame(i as f64 * 0.01, 5.0)).collect();
        assert_eq!(settling_time_s(&frames, &FlightTarget::altitude(5.0)), 0.0);
    }

    #[test]
    fn settling_time_when_at_end_in_threshold() {
        // Pierwsze 50 klatek daleko od celu, potem w progu
        let mut frames: Vec<_> = (0..50).map(|i| frame(i as f64 * 0.01, 0.0)).collect();
        let settled: Vec<_> = (50..100).map(|i| frame(i as f64 * 0.01, 5.0)).collect();
        frames.extend(settled);
        // Ostatnie przekroczenie to klatka 49 → time = 0.49s
        let st = settling_time_s(&frames, &FlightTarget::altitude(5.0));
        assert!((st - 0.49).abs() < 1e-10, "settling_time={}", st);
    }
}
