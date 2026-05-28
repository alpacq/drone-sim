use crate::scenario::{Axis, MetricKind};
use drone_control::target::FlightTarget;
use drone_model::math::euler::quat_to_euler;
use drone_sim::runner::SimFrame;

pub fn compute(metric: &MetricKind, frames: &[SimFrame], target: &FlightTarget) -> f64 {
    let target_pos = target.position.unwrap_or_default();
    match metric {
        MetricKind::PositionRms3d => position_rms_3d(frames, target),
        MetricKind::PositionRmsAxis(axis) => position_rms_axis(frames, target, axis),
        MetricKind::PositionMaxError3d => position_max_error_3d(frames, target),
        MetricKind::PositionMaxErrorAxis(axis) => position_max_error_axis(frames, target, axis),
        MetricKind::VelocityRms3d => velocity_rms_3d(frames),
        MetricKind::VelocityRmsAxis(axis) => velocity_rms_axis(frames, axis),
        MetricKind::AttitudeRms => attitude_rms(frames),
        MetricKind::AttitudeMaxError => attitude_max_error(frames),
        MetricKind::OvershootPercent => overshoot_percent(frames, target),
        MetricKind::SettlingTimeS => settling_time_s(frames, target),
        MetricKind::RiseTimeS => rise_time_s(frames, target_pos.z),
        MetricKind::SteadyStateError => steady_state_error(frames, target_pos.z),
        MetricKind::ControlEnergy => control_energy(frames),
        MetricKind::MaxControlRate => max_control_rate(frames),
    }
}

// ── 3D / per-axis position metrics ────────────────────────────────────────

/// RMS of 3D position error: √( mean(||p_i - p_target||²) )
pub fn position_rms_3d(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    if frames.is_empty() {
        return 0.0;
    }
    let t = target.position.unwrap_or_default();
    let sum_sq: f64 = frames
        .iter()
        .map(|f| {
            let d = f.state.position - t;
            d.norm_squared()
        })
        .sum();
    (sum_sq / frames.len() as f64).sqrt()
}

/// RMS of position error along one axis.
pub fn position_rms_axis(frames: &[SimFrame], target: &FlightTarget, axis: &Axis) -> f64 {
    if frames.is_empty() {
        return 0.0;
    }
    let t = target.position.unwrap_or_default();
    let sum_sq: f64 = frames
        .iter()
        .map(|f| {
            let d = match axis {
                Axis::X => f.state.position.x - t.x,
                Axis::Y => f.state.position.y - t.y,
                Axis::Z => f.state.position.z - t.z,
            };
            d * d
        })
        .sum();
    (sum_sq / frames.len() as f64).sqrt()
}

/// Maximum 3D position error: max(||p_i - p_target||)
pub fn position_max_error_3d(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    let t = target.position.unwrap_or_default();
    frames
        .iter()
        .map(|f| (f.state.position - t).norm())
        .fold(0.0_f64, f64::max)
}

/// Maximum position error along one axis.
pub fn position_max_error_axis(frames: &[SimFrame], target: &FlightTarget, axis: &Axis) -> f64 {
    let t = target.position.unwrap_or_default();
    frames
        .iter()
        .map(|f| {
            (match axis {
                Axis::X => f.state.position.x - t.x,
                Axis::Y => f.state.position.y - t.y,
                Axis::Z => f.state.position.z - t.z,
            })
            .abs()
        })
        .fold(0.0_f64, f64::max)
}

// ── Velocity metrics (reference = 0, i.e. deviation from rest) ────────────

/// RMS of 3D velocity magnitude.
pub fn velocity_rms_3d(frames: &[SimFrame]) -> f64 {
    if frames.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = frames.iter().map(|f| f.state.velocity.norm_squared()).sum();
    (sum_sq / frames.len() as f64).sqrt()
}

/// RMS of velocity along one axis.
pub fn velocity_rms_axis(frames: &[SimFrame], axis: &Axis) -> f64 {
    if frames.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = frames
        .iter()
        .map(|f| {
            let v = match axis {
                Axis::X => f.state.velocity.x,
                Axis::Y => f.state.velocity.y,
                Axis::Z => f.state.velocity.z,
            };
            v * v
        })
        .sum();
    (sum_sq / frames.len() as f64).sqrt()
}

// ── Attitude metrics (reference = level flight, roll=0 pitch=0) ────────────

/// RMS of attitude error (roll² + pitch²) in radians.
pub fn attitude_rms(frames: &[SimFrame]) -> f64 {
    if frames.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = frames
        .iter()
        .map(|f| {
            let e = quat_to_euler(&f.state.orientation);
            e.roll * e.roll + e.pitch * e.pitch
        })
        .sum();
    (sum_sq / frames.len() as f64).sqrt()
}

/// Maximum attitude error (max of √(roll²+pitch²)) in radians.
pub fn attitude_max_error(frames: &[SimFrame]) -> f64 {
    frames
        .iter()
        .map(|f| {
            let e = quat_to_euler(&f.state.orientation);
            (e.roll * e.roll + e.pitch * e.pitch).sqrt()
        })
        .fold(0.0_f64, f64::max)
}

// ── Legacy Z-only helpers (kept for direct use in comparison.rs) ───────────

/// Root Mean Square of z position error
/// RMS = √( (1/N) × Σ(z_i - z_target)² )
pub fn position_rms_z(frames: &[SimFrame], target: &FlightTarget) -> f64 {
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
pub fn position_max_error_z(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    frames
        .iter()
        .map(|f| (f.state.position.z - target.position.unwrap_or_default().z).abs())
        .fold(0.0_f64, f64::max)
}

/// overshoot - how much percent drone overreached target
pub fn overshoot_percent(frames: &[SimFrame], target: &FlightTarget) -> f64 {
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
pub fn settling_time_s(frames: &[SimFrame], target: &FlightTarget) -> f64 {
    let threshold = 0.1;
    let last_violation = frames.iter().enumerate().rev().find(|(_, f)| {
        (f.state.position.z - target.position.unwrap_or_default().z).abs() >= threshold
    });

    match last_violation {
        Some((idx, _)) => frames[idx].time,
        None => 0.0,
    }
}

/// Compute a proxy for total energy consumed by the motors.
///
/// In the propeller model: torque τ = k_torque·ω², so mechanical power is
/// P = τ·ω = k_torque·ω³.  Summing ω³ over all motors and integrating over
/// time gives a quantity proportional to energy (k_torque cancels when
/// comparing controllers on the same vehicle model).
///
/// Using ω² (the naive choice) would give a different relative ordering for
/// profiles that mix high-RPM short bursts vs. sustained moderate RPM.
pub fn control_energy(frames: &[SimFrame]) -> f64 {
    if frames.len() < 2 {
        return 0.0;
    }

    frames
        .windows(2)
        .map(|w| {
            let dt = w[1].time - w[0].time;
            // Power proxy: sum of ω³ ∝ mechanical power (P = k_torque · ω³)
            let power_proxy = match &w[0].state.actuator_state {
                Some(drone_model::state::ActuatorState::QuadrotorMotors(speeds)) => {
                    use drone_model::motor::Motor;
                    Motor::ALL.iter().map(|&m| speeds[m].powi(3)).sum::<f64>()
                }
                None => 0.0,
            };
            power_proxy * dt
        })
        .sum()
}

pub fn rise_time_s(frames: &[SimFrame], target_z: f64) -> f64 {
    let initial_z = frames.first().map(|f| f.state.position.z).unwrap_or(0.0);
    let total_change = target_z - initial_z;
    if total_change.abs() < 1e-6 {
        return 0.0;
    }

    let z_10 = initial_z + 0.10 * total_change;
    let z_90 = initial_z + 0.90 * total_change;

    let t_10 = frames
        .iter()
        .find(|f| (f.state.position.z - initial_z).abs() >= (z_10 - initial_z).abs())
        .map(|f| f.time)
        .unwrap_or(f64::INFINITY);

    let t_90 = frames
        .iter()
        .find(|f| (f.state.position.z - initial_z).abs() >= (z_90 - initial_z).abs())
        .map(|f| f.time)
        .unwrap_or(f64::INFINITY);

    if t_90 == f64::INFINITY {
        f64::INFINITY
    } else {
        t_90 - t_10
    }
}

pub fn steady_state_error(frames: &[SimFrame], target_z: f64) -> f64 {
    if frames.is_empty() {
        return 0.0;
    }
    let cutoff = frames.last().unwrap().time * 0.8;
    let late_frames: Vec<_> = frames.iter().filter(|f| f.time >= cutoff).collect();
    if late_frames.is_empty() {
        return 0.0;
    }

    late_frames
        .iter()
        .map(|f| (f.state.position.z - target_z).abs())
        .sum::<f64>()
        / late_frames.len() as f64
}

pub fn max_control_rate(frames: &[SimFrame]) -> f64 {
    frames
        .windows(2)
        .map(|w| {
            let dt = (w[1].time - w[0].time).max(1e-10);
            match (&w[0].state.actuator_state, &w[1].state.actuator_state) {
                (
                    Some(drone_model::state::ActuatorState::QuadrotorMotors(s0)),
                    Some(drone_model::state::ActuatorState::QuadrotorMotors(s1)),
                ) => {
                    use drone_model::motor::Motor;
                    Motor::ALL
                        .iter()
                        .map(|&m| ((s1[m] - s0[m]) / dt).abs())
                        .fold(0.0_f64, f64::max)
                }
                _ => 0.0,
            }
        })
        .fold(0.0_f64, f64::max)
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

    #[test]
    fn rise_time_monotonically_rising() {
        // Linear growth from 0 to 5m in 5s
        let frames: Vec<_> = (0..=100)
            .map(|i| frame(i as f64 * 0.05, i as f64 * 0.05))
            .collect();
        let rt = rise_time_s(&frames, 5.0);
        // 10%=0.5m at t≈0.5s, 90%=4.5m at t≈4.5s → rise≈4.0s
        assert!(rt > 3.0 && rt < 5.0, "Rise time = {:.2}s", rt);
    }

    #[test]
    fn steady_state_error_zero_at_target() {
        let frames: Vec<_> = (0..100).map(|i| frame(i as f64 * 0.1, 5.0)).collect();
        let sse = steady_state_error(&frames, 5.0);
        assert!(sse < 1e-10);
    }
}
