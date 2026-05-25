use super::Disturbance;
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct WindGustConfig {
    pub at_s: f64, // [s]
    #[serde(default = "default_duration")]
    pub duration_s: f64, // [s]
    pub force: [f64; 3], // [N]
}

fn default_duration() -> f64 {
    0.1
}

pub struct WindGust {
    start_s: f64,
    end_s: f64,
    force: Vector3<f64>,
}

impl WindGust {
    pub fn from_config(c: WindGustConfig) -> Self {
        Self {
            start_s: c.at_s,
            end_s: c.at_s + c.duration_s,
            force: Vector3::from(c.force),
        }
    }
}

impl Disturbance for WindGust {
    fn is_active(&self, time: f64) -> bool {
        time >= self.start_s && time < self.end_s
    }

    fn apply(&self, state: &mut DroneState, model: &dyn VehicleModel, dt: TimeStep) {
        let delta_v = self.force * dt.seconds() / model.mass();
        state.velocity += delta_v;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{state::DroneState, time::TimeStep};
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn active_in_range() {
        let gust = WindGust {
            start_s: 2.0,
            end_s: 2.5,
            force: Vector3::new(1.0, 0.0, 0.0),
        };
        assert!(!gust.is_active(1.9));
        assert!(gust.is_active(2.0));
        assert!(gust.is_active(2.25));
        assert!(gust.is_active(2.49));
        assert!(!gust.is_active(2.5));
    }

    #[test]
    fn applies_force_as_velocity_change() {
        use drone_model::vehicle::quadrotor::QuadrotorModel;

        let model = QuadrotorModel::mini3();
        let gust = WindGust {
            start_s: 0.0,
            end_s: 1.0,
            force: Vector3::new(2.0, 0.0, 0.0),
        };
        let mut state = DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        };
        let dt = TimeStep::constant(0.01);
        gust.apply(&mut state, &model, dt);

        let expected = 2.0 * 0.01 / 0.249;
        assert!((state.velocity.x - expected).abs() < 1e-6);
    }
}
