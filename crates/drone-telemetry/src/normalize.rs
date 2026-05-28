use crate::frame::TelemetryFrame;
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct TrajectoryPoint {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Option<Vector3<f64>>,
    pub frame_idx: u32,
}

#[derive(Debug, Clone)]
pub struct FlightTrajectory {
    pub points: Vec<TrajectoryPoint>,
    pub origin: GpsOrigin,
    pub duration_s: f64,
}

impl FlightTrajectory {
    pub fn len(&self) -> usize {
        self.points.len()
    }
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GpsOrigin {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

pub fn normalize(frames: &[TelemetryFrame]) -> Result<FlightTrajectory, crate::error::TelemetryError> {
    let gps_frames: Vec<&TelemetryFrame> = frames
        .iter()
        .filter(|f| f.has_gps() && f.rel_alt.is_some())
        .collect();

    if gps_frames.is_empty() {
        return Err(crate::error::TelemetryError::NoGpsFrames);
    }
    if gps_frames.len() < 3 {
        return Err(crate::error::TelemetryError::NotEnoughGpsFrames {
            found: gps_frames.len(),
        });
    }

    let first = gps_frames[0];
    let origin = GpsOrigin {
        latitude: first.latitude.unwrap(),
        longitude: first.longitude.unwrap(),
        altitude: first.rel_alt.unwrap() as f64,
    };

    let mut raw_positions: Vec<(f64, Vector3<f64>)> = Vec::new();
    let mut cumulative_time = 0.0_f64;

    for frame in &gps_frames {
        let pos = gps_to_enu(
            frame.latitude.unwrap(),
            frame.longitude.unwrap(),
            frame.rel_alt.unwrap() as f64,
            &origin,
        );
        raw_positions.push((cumulative_time, pos));
        cumulative_time += frame.dt_seconds();
    }

    let n = raw_positions.len();
    let mut points = Vec::with_capacity(n);

    for i in 0..n {
        let (time, pos) = raw_positions[i];

        let velocity = if n < 2 {
            None
        } else if i == 0 {
            let dt = raw_positions[1].0 - raw_positions[0].0;
            if dt > 1e-6 {
                Some((raw_positions[1].1 - raw_positions[0].1) / dt)
            } else {
                None
            }
        } else if i == n - 1 {
            let dt = raw_positions[n - 1].0 - raw_positions[n - 2].0;
            if dt > 1e-6 {
                Some((raw_positions[n - 1].1 - raw_positions[n - 2].1) / dt)
            } else {
                None
            }
        } else {
            let dt = raw_positions[i + 1].0 - raw_positions[i - 1].0;
            if dt > 1e-6 {
                Some((raw_positions[i + 1].1 - raw_positions[i - 1].1) / dt)
            } else {
                None
            }
        };

        points.push(TrajectoryPoint {
            time,
            position: pos,
            velocity,
            frame_idx: gps_frames[i].index,
        });
    }

    let duration_s = points.last().map(|p| p.time).unwrap_or(0.0);

    Ok(FlightTrajectory {
        points,
        origin,
        duration_s,
    })
}

pub fn gps_to_enu(lat: f64, lon: f64, alt: f64, origin: &GpsOrigin) -> Vector3<f64> {
    const METERS_PER_DEGREE: f64 = 111_320.0;

    let dlat = lat - origin.latitude;
    let dlon = lon - origin.longitude;
    let dalt = alt - origin.altitude;

    let x = dlon * origin.latitude.to_radians().cos() * METERS_PER_DEGREE;
    let y = dlat * METERS_PER_DEGREE;
    let z = dalt;

    Vector3::new(x, y, z)
}

pub fn enu_to_gps(enu: &Vector3<f64>, origin: &GpsOrigin) -> (f64, f64, f64) {
    const METERS_PER_DEGREE: f64 = 111_320.0;

    let dlat = enu.y / METERS_PER_DEGREE;
    let dlon = enu.x / (METERS_PER_DEGREE * origin.latitude.to_radians().cos());

    (
        origin.latitude + dlat,
        origin.longitude + dlon,
        origin.altitude + enu.z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::TelemetryFrame;

    fn make_frame(idx: u32, lat: f64, lon: f64, alt: f32) -> TelemetryFrame {
        TelemetryFrame {
            index: idx,
            timestamp: None,
            duration_ms: 33,
            latitude: Some(lat),
            longitude: Some(lon),
            rel_alt: Some(alt),
            abs_alt: None,
            gimbal_yaw: None,
            gimbal_pitch: None,
            gimbal_roll: None,
            iso: None,
            shutter: None,
            fnum: None,
            color_temp: None,
        }
    }

    #[test]
    fn first_point_in_origin() {
        let frames = vec![
            make_frame(1, 52.0, 21.0, 0.0),
            make_frame(2, 52.001, 21.001, 5.0),
            make_frame(3, 52.002, 21.002, 10.0),
        ];
        let traj = normalize(&frames).unwrap();
        assert!(
            traj.points[0].position.norm() < 1e-6,
            "First point: {:?}",
            traj.points[0].position
        );
    }

    #[test]
    fn altitude_is_z_axis() {
        let frames = vec![
            make_frame(1, 52.0, 21.0, 0.0),
            make_frame(2, 52.0, 21.0, 10.0),
            make_frame(3, 52.0, 21.0, 20.0),
        ];
        let traj = normalize(&frames).unwrap();
        assert!(
            (traj.points[1].position.z - 10.0).abs() < 0.01,
            "z = {}",
            traj.points[1].position.z
        );
        assert!(traj.points[1].position.x.abs() < 0.01);
        assert!(traj.points[1].position.y.abs() < 0.01);
    }

    #[test]
    fn horizontal_velocity_correct() {
        let frames = vec![
            make_frame(1, 52.0, 21.0, 0.0),
            make_frame(2, 52.0, 21.0, 5.0),
            make_frame(3, 52.0, 21.0, 10.0),
        ];
        let traj = normalize(&frames).unwrap();
        let vz = traj.points[1].velocity.unwrap().z;
        let expected = 10.0 / (2.0 * 0.033);
        assert!(
            (vz - expected).abs() < 1.0,
            "vz = {:.2}, expected {:.2}",
            vz,
            expected
        );
    }

    #[test]
    fn round_trip_gps_enu() {
        let origin = GpsOrigin {
            latitude: 52.237049,
            longitude: 21.017532,
            altitude: 0.0,
        };
        let lat = 52.237500;
        let lon = 21.018000;
        let alt = 15.0;

        let enu = gps_to_enu(lat, lon, alt, &origin);
        let (lat2, lon2, alt2) = enu_to_gps(&enu, &origin);

        assert!((lat - lat2).abs() < 1e-8);
        assert!((lon - lon2).abs() < 1e-8);
        assert!((alt - alt2).abs() < 1e-6);
    }

    #[test]
    fn movement_to_east_is_positive_x() {
        let origin = GpsOrigin {
            latitude: 52.0,
            longitude: 21.0,
            altitude: 0.0,
        };
        let enu = gps_to_enu(52.0, 21.001, 0.0, &origin);
        assert!(enu.x > 0.0, "East → x > 0: x = {:.2}m", enu.x);
        assert!(enu.y.abs() < 0.1);
    }

    #[test]
    fn movement_to_north_is_positive_y() {
        let origin = GpsOrigin {
            latitude: 52.0,
            longitude: 21.0,
            altitude: 0.0,
        };
        let enu = gps_to_enu(52.001, 21.0, 0.0, &origin);
        assert!(enu.y > 0.0, "North → y > 0: y = {:.2}m", enu.y);
        assert!(enu.x.abs() < 0.1);
    }
}
