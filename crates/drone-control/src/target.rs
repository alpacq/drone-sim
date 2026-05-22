use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct FlightTarget {
    // target position in world frame [m]
    pub position: Option<Vector3<f64>>,
    // target yaw angle [rad]
    pub yaw: Option<f64>,
}

impl FlightTarget {
    pub fn altitude(z: f64) -> Self {
        Self {
            position: Some(Vector3::new(0.0, 0.0, z)),
            yaw: None,
        }
    }

    pub fn position(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: Some(Vector3::new(x, y, z)),
            yaw: None,
        }
    }

    pub fn full(x: f64, y: f64, z: f64, yaw: f64) -> Self {
        Self {
            position: Some(Vector3::new(x, y, z)),
            yaw: Some(yaw),
        }
    }
}
