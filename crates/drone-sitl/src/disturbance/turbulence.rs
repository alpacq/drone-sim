use super::Disturbance;
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use nalgebra::Vector3;
use rand::{SeedableRng, rngs::SmallRng};
use rand_distr::{Distribution, Normal};
use serde::Deserialize;
use std::sync::Mutex;

#[derive(Debug, Clone, Deserialize)]
pub struct TurbulenceConfig {
    pub start_s: f64,     // [s]
    pub end_s: f64,       // [s]
    pub intensity_n: f64, // [N]
    #[serde(default)]
    pub seed: u64,
    /// Apply turbulence only on the Z axis. X/Y are undisturbed.
    /// Use this when you want to test altitude rejection without XY→Z coupling effects.
    #[serde(default)]
    pub z_only: bool,
}

pub struct Turbulence {
    start_s: f64,
    end_s: f64,
    normal: Normal<f64>, // normal distribution N(0, intensity_n^2)
    rng: Mutex<SmallRng>,
    z_only: bool,
}

impl Turbulence {
    pub fn from_config(config: TurbulenceConfig) -> Self {
        let normal = Normal::new(0.0, config.intensity_n).expect("intensity_n must be > 0");

        Self {
            start_s: config.start_s,
            end_s: config.end_s,
            normal,
            rng: Mutex::new(SmallRng::seed_from_u64(config.seed)),
            z_only: config.z_only,
        }
    }

    fn sample_force(&self) -> f64 {
        let mut rng = self.rng.lock().expect("Mutex shouldn't be poisoned");
        self.normal.sample(&mut *rng)
    }
}

impl Disturbance for Turbulence {
    fn is_active(&self, time: f64) -> bool {
        time >= self.start_s && time < self.end_s
    }

    fn apply(&self, state: &mut DroneState, model: &dyn VehicleModel, dt: TimeStep) {
        let fz = self.sample_force();
        let force = if self.z_only {
            Vector3::new(0.0, 0.0, fz)
        } else {
            Vector3::new(self.sample_force(), self.sample_force(), fz)
        };
        let delta_v = force * dt.seconds() / model.mass();
        state.velocity += delta_v;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{state::DroneState, time::TimeStep, vehicle::quadrotor::QuadrotorModel};
    use nalgebra::{UnitQuaternion, Vector3};

    fn make_turbulence(intensity: f64, seed: u64) -> Turbulence {
        Turbulence::from_config(TurbulenceConfig {
            start_s: 0.0,
            end_s: 100.0,
            intensity_n: intensity,
            seed,
            z_only: false,
        })
    }

    fn ground_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    #[test]
    fn active_in_range() {
        let t = make_turbulence(0.5, 42);
        assert!(!t.is_active(-0.1));
        assert!(t.is_active(0.0));
        assert!(t.is_active(50.0));
        assert!(!t.is_active(100.0));
    }

    #[test]
    fn the_same_seed_gives_the_same_sequence() {
        // Determinism is critical for reproducible tests.
        let model = QuadrotorModel::mini3();
        let dt = TimeStep::constant(0.01);

        let apply_n = |seed: u64, n: usize| -> Vector3<f64> {
            let t = make_turbulence(1.0, seed);
            let mut s = ground_state();
            for _ in 0..n {
                t.apply(&mut s, &model, dt);
            }
            s.velocity
        };

        let v1 = apply_n(42, 100);
        let v2 = apply_n(42, 100);
        let v3 = apply_n(99, 100); // different seed

        assert_eq!(v1, v2, "Same seed should give the same sequence");

        assert_ne!(v1, v3, "Different seeds should give different sequences");
    }

    #[test]
    fn mean_near_zero_gaussian() {
        // With a large sample: E[X] ≈ 0 dla N(0, σ²)
        // Law of large numbers — 10000 samples should give μ < 0.05σ
        let model = QuadrotorModel::mini3();
        let dt = TimeStep::constant(0.01);
        let t = make_turbulence(1.0, 42);
        let mut total_vx = 0.0_f64;
        let n = 10_000_usize;

        for _ in 0..n {
            let mut s = ground_state();
            t.apply(&mut s, &model, dt);
            total_vx += s.velocity.x;
        }

        let mean_vx = total_vx / n as f64;
        // For N(0,1) standard deviation of mean = σ/√n = 1/100 = 0.01
        // Tolerate 5σ = 0.05 (probability of exceeding ~0.00006%)
        assert!(
            mean_vx.abs() < 0.05,
            "Mean should be close to 0, got: {:.4}",
            mean_vx
        );
    }

    #[test]
    fn variance_consistent_with_intensity() {
        // Var[Δv] = (intensity · dt / m)²
        // Std[Δv] ≈ intensity · dt / m
        let model = QuadrotorModel::mini3();
        let dt = TimeStep::constant(0.01);
        let intensity = 1.0_f64;
        let t = make_turbulence(intensity, 42);
        let n = 50_000_usize;

        let mut sum_sq = 0.0_f64;
        for _ in 0..n {
            let mut s = ground_state();
            t.apply(&mut s, &model, dt);
            sum_sq += s.velocity.x * s.velocity.x;
        }

        let std_empirical = (sum_sq / n as f64).sqrt();
        let std_expected = intensity * dt.seconds() / model.mass();

        // Tolerance 5% — with 50000 samples, standard deviation of mean σ/√n is small
        let rel_error = (std_empirical - std_expected).abs() / std_expected;
        assert!(
            rel_error < 0.05,
            "Std empirical={:.5}, expected={:.5}, error={:.1}%",
            std_empirical,
            std_expected,
            rel_error * 100.0
        );
    }
}
