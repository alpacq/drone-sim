use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use serde::Deserialize;

pub mod motor_failure;
pub mod turbulence;
pub mod wind_gust;

pub use motor_failure::MotorFailure;
pub use turbulence::Turbulence;
pub use wind_gust::WindGust;

pub trait Disturbance: Send + Sync {
    fn is_active(&self, time: f64) -> bool;

    fn apply(&self, state: &mut DroneState, model: &dyn VehicleModel, dt: TimeStep);
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum DisturbanceConfig {
    WindGust(wind_gust::WindGustConfig),
    Turbulence(turbulence::TurbulenceConfig),
    MotorFailure(motor_failure::MotorFailureConfig),
}

impl DisturbanceConfig {
    pub fn into_disturbance(self) -> Box<dyn Disturbance> {
        match self {
            Self::WindGust(c) => Box::new(WindGust::from_config(c)),
            Self::Turbulence(c) => Box::new(Turbulence::from_config(c)),
            Self::MotorFailure(c) => Box::new(MotorFailure::from_config(c)),
        }
    }
}
