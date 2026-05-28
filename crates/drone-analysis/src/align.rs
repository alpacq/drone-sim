use drone_sim::runner::SimFrame;
use drone_telemetry::normalize::FlightTrajectory;
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct AlignedPoint {
    pub time: f64,
    pub sim_pos: Vector3<f64>,
    pub sim_vel: Vector3<f64>,
    pub telem_pos: Vector3<f64>,
    pub telem_vel: Vector3<f64>,
    pub pos_error: f64,
    pub vel_error: f64,
}

pub struct AlignedTrajectory {
    pub points: Vec<AlignedPoint>,
    pub duration_s: f64,
}

impl AlignedTrajectory {
    pub fn position_rms(&self) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = self.points.iter().map(|p| p.pos_error * p.pos_error).sum();
        (sum_sq / self.points.len() as f64).sqrt()
    }

    pub fn position_max(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.pos_error)
            .fold(0.0_f64, f64::max)
    }

    pub fn velocity_rms(&self) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = self.points.iter().map(|p| p.vel_error * p.vel_error).sum();
        (sum_sq / self.points.len() as f64).sqrt()
    }

    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "time,sim_x,sim_y,sim_z,telem_x,telem_y,telem_z,\
             pos_error,vel_error\n",
        );
        for p in &self.points {
            out.push_str(&format!(
                "{:.4},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4}\n",
                p.time,
                p.sim_pos.x,
                p.sim_pos.y,
                p.sim_pos.z,
                p.telem_pos.x,
                p.telem_pos.y,
                p.telem_pos.z,
                p.pos_error,
                p.vel_error,
            ));
        }
        out
    }
}

pub fn align(sim_frames: &[SimFrame], telemetry: &FlightTrajectory) -> AlignedTrajectory {
    let mut points = Vec::new();

    for telem_pt in &telemetry.points {
        let t = telem_pt.time;

        if let Some((sim_pos, sim_vel)) = interpolate_sim(sim_frames, t) {
            let pos_error = (sim_pos - telem_pt.position).norm();
            let vel_error = match telem_pt.velocity {
                Some(tv) => (sim_vel - tv).norm(),
                None => 0.0,
            };

            points.push(AlignedPoint {
                time: t,
                sim_pos,
                sim_vel,
                telem_pos: telem_pt.position,
                telem_vel: telem_pt.velocity.unwrap_or(Vector3::zeros()),
                pos_error,
                vel_error,
            });
        }
    }

    let duration_s = points.last().map(|p| p.time).unwrap_or(0.0);
    AlignedTrajectory { points, duration_s }
}

fn interpolate_sim(frames: &[SimFrame], t: f64) -> Option<(Vector3<f64>, Vector3<f64>)> {
    if frames.is_empty() {
        return None;
    }

    if t <= frames[0].time {
        return Some((frames[0].state.position, frames[0].state.velocity));
    }
    if t >= frames.last().unwrap().time {
        let last = frames.last().unwrap();
        return Some((last.state.position, last.state.velocity));
    }

    let idx = frames.partition_point(|f| f.time <= t);
    if idx == 0 || idx >= frames.len() {
        return None;
    }

    let f0 = &frames[idx - 1];
    let f1 = &frames[idx];
    let dt = f1.time - f0.time;

    if dt < 1e-10 {
        return Some((f0.state.position, f0.state.velocity));
    }

    let alpha = (t - f0.time) / dt;
    let pos = f0.state.position + (f1.state.position - f0.state.position) * alpha;
    let vel = f0.state.velocity + (f1.state.velocity - f0.state.velocity) * alpha;

    Some((pos, vel))
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::state::DroneState;
    use drone_telemetry::normalize::TrajectoryPoint;
    use nalgebra::{UnitQuaternion, Vector3};

    fn sim_frame(time: f64, z: f64) -> SimFrame {
        SimFrame {
            time,
            state: DroneState {
                position: Vector3::new(0.0, 0.0, z),
                velocity: Vector3::new(0.0, 0.0, 1.0),
                orientation: UnitQuaternion::identity(),
                angular_velocity: Vector3::zeros(),
                actuator_state: None,
            },
        }
    }

    fn telem_point(time: f64, z: f64) -> TrajectoryPoint {
        TrajectoryPoint {
            time,
            position: Vector3::new(0.0, 0.0, z),
            velocity: Some(Vector3::new(0.0, 0.0, 1.0)),
            frame_idx: 0,
        }
    }

    #[test]
    fn linear_interpolation() {
        let frames = vec![sim_frame(0.0, 0.0), sim_frame(1.0, 10.0)];
        let (pos, _) = interpolate_sim(&frames, 0.5).unwrap();
        assert!(
            (pos.z - 5.0).abs() < 1e-10,
            "Interpolation in the middle: z = {}",
            pos.z
        );
    }

    #[test]
    fn align_of_identical_trajectories_yields_error() {
        let sim = vec![
            sim_frame(0.0, 0.0),
            sim_frame(0.033, 1.0),
            sim_frame(0.066, 2.0),
        ];
        let telem = FlightTrajectory {
            points: vec![
                telem_point(0.0, 0.0),
                telem_point(0.033, 1.0),
                telem_point(0.066, 2.0),
            ],
            origin: drone_telemetry::normalize::GpsOrigin {
                latitude: 52.0,
                longitude: 21.0,
                altitude: 0.0,
            },
            duration_s: 0.066,
        };

        let aligned = align(&sim, &telem);
        assert!(
            aligned.position_rms() < 0.01,
            "Identical trajectories → error ≈ 0: {:.4}",
            aligned.position_rms()
        );
    }
}
