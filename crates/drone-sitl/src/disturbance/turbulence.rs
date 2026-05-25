use super::Disturbance;
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct TurbulenceConfig {
    pub start_s: f64,     // [s]
    pub end_s: f64,       // [s]
    pub intensity_n: f64, // [N]
    #[serde(default)]
    pub seed: u64,
}

pub struct Turbulence {
    start_s: f64,
    end_s: f64,
    intensity_n: f64,
    seed: u64,
    step: std::sync::atomic::AtomicU64,
}

impl Turbulence {
    pub fn from_config(config: TurbulenceConfig) -> Self {
        Self {
            start_s: config.start_s,
            end_s: config.end_s,
            intensity_n: config.intensity_n,
            seed: config.seed,
            step: std::sync::atomic::AtomicU64::new(0),
        }
    }

    fn pseudo_noise(&self, axis: u64) -> f64 {
        let step = self.step.load(std::sync::atomic::Ordering::Relaxed);
        let mut x = self
            .seed
            .wrapping_add(step.wrapping_mul(2654435761))
            .wrapping_add(axis.wrapping_mul(40503));

        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        (x as f64 / u64::MAX as f64) * 2.0 - 1.0
    }
}

impl Disturbance for Turbulence {
    fn is_active(&self, time: f64) -> bool {
        time >= self.start_s && time <= self.end_s
    }

    fn apply(&self, state: &mut DroneState, model: &dyn VehicleModel, dt: TimeStep) {
        let force = Vector3::new(
            self.pseudo_noise(0) * self.intensity_n,
            self.pseudo_noise(1) * self.intensity_n,
            self.pseudo_noise(2) * self.intensity_n,
        );
        let delta_v = force * dt.seconds() / model.mass();
        state.velocity += delta_v;

        self.step.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
}
