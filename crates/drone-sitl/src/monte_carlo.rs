//! Monte Carlo batch simulation with parallel execution.
//!
//! Runs a scenario many times with perturbed initial conditions and
//! aggregates per-metric statistics (mean, std-dev, min, max, pass rate).

use rand::SeedableRng;
use rand::rngs::SmallRng;
use rand_distr::{Distribution, Normal};
use rayon::prelude::*;

use drone_model::vehicle::VehicleModel;

use crate::report::AssertionResult;
use crate::runner::{ControllerFactory, run_scenario};
use crate::scenario::Scenario;

/// Configuration for a Monte Carlo sweep.
#[derive(Debug, Clone)]
pub struct MonteCarloConfig {
    /// Number of independent simulation runs.
    pub runs: usize,
    /// Std-dev of Gaussian position perturbation added to initial conditions [m].
    pub pos_noise_m: f64,
    /// Std-dev of Gaussian velocity perturbation [m/s].
    pub vel_noise_ms: f64,
    /// Base random seed; run `i` uses seed `base_seed + i` for reproducibility.
    pub base_seed: u64,
}

impl Default for MonteCarloConfig {
    fn default() -> Self {
        Self {
            runs: 100,
            pos_noise_m: 0.5,
            vel_noise_ms: 0.1,
            base_seed: 42,
        }
    }
}

/// Per-metric statistics over all Monte Carlo runs.
#[derive(Debug, Clone)]
pub struct MetricStats {
    /// Metric name (e.g. `"OvershootPercent"`).
    pub name: String,
    /// Assertion threshold from the scenario.
    pub threshold: f64,
    /// Mean value across all runs.
    pub mean: f64,
    /// Standard deviation across all runs.
    pub std_dev: f64,
    /// Minimum value observed.
    pub min: f64,
    /// Maximum value observed.
    pub max: f64,
    /// Fraction of runs where metric ≤ threshold.
    pub pass_rate: f64,
}

/// Full report from a Monte Carlo sweep.
#[derive(Debug, Clone)]
pub struct MonteCarloReport {
    /// Name of the scenario that was swept.
    pub scenario_name: String,
    /// Number of runs executed.
    pub runs: usize,
    /// Per-metric aggregated statistics.
    pub metrics: Vec<MetricStats>,
}

impl MonteCarloReport {
    /// Serialize report to CSV.
    /// Columns: `metric,threshold,mean,std_dev,min,max,pass_rate`
    pub fn to_csv(&self) -> String {
        let mut out =
            String::from("metric,threshold,mean,std_dev,min,max,pass_rate\n");
        for m in &self.metrics {
            out.push_str(&format!(
                "{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.4}\n",
                m.name, m.threshold, m.mean, m.std_dev, m.min, m.max, m.pass_rate,
            ));
        }
        out
    }

    /// Pretty-print the report as a table.
    pub fn print(&self) {
        println!("\n╔══════════════════════════════════════════════════════════════════════════╗");
        println!(
            "║  Monte Carlo: {}  ({} runs)",
            self.scenario_name, self.runs
        );
        println!("╠══════════════════════════════════════════════════════════════════════════╣");
        println!(
            "║ {:<24} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
            "Metric", "Mean", "StdDev", "Min", "Max", "Thresh", "Pass%"
        );
        println!("╠══════════════════════════════════════════════════════════════════════════╣");
        for m in &self.metrics {
            println!(
                "║ {:<24} {:>8.3} {:>8.3} {:>8.3} {:>8.3} {:>8.3} {:>7.1}%",
                m.name,
                m.mean,
                m.std_dev,
                m.min,
                m.max,
                m.threshold,
                m.pass_rate * 100.0,
            );
        }
        println!("╚══════════════════════════════════════════════════════════════════════════╝");
    }
}

/// Run the scenario `cfg.runs` times in parallel.
///
/// Each run independently perturbs the initial state with Gaussian noise
/// seeded from `cfg.base_seed + run_index`.
///
/// The `ControllerFactory` has `Send + Sync` bounds, so Rayon can call it
/// from multiple threads.
pub fn run_monte_carlo(
    scenario: &Scenario,
    model: &(dyn VehicleModel + Send + Sync),
    factory: &ControllerFactory,
    cfg: &MonteCarloConfig,
) -> MonteCarloReport {
    let all_results: Vec<Vec<AssertionResult>> = (0..cfg.runs)
        .into_par_iter()
        .map(|i| {
            let mut rng = SmallRng::seed_from_u64(cfg.base_seed + i as u64);
            let pos_noise = Normal::new(0.0, cfg.pos_noise_m).unwrap();
            let vel_noise = Normal::new(0.0, cfg.vel_noise_ms).unwrap();

            let mut scenario_run = scenario.clone();
            let ic = &mut scenario_run.initial;
            ic.position[0] += pos_noise.sample(&mut rng);
            ic.position[1] += pos_noise.sample(&mut rng);
            // Keep z >= 0 by taking abs of the perturbation.
            let z_perturb: f64 = pos_noise.sample(&mut rng);
            ic.position[2] += z_perturb.abs();
            ic.velocity[0] += vel_noise.sample(&mut rng);
            ic.velocity[1] += vel_noise.sample(&mut rng);
            ic.velocity[2] += vel_noise.sample(&mut rng);

            match run_scenario(&scenario_run, model, factory) {
                Ok(report) => report.assertions,
                Err(_) => scenario
                    .assertions
                    .iter()
                    .map(|a| AssertionResult {
                        metric: a.metric.to_string(),
                        value: f64::INFINITY,
                        max: a.max,
                        passed: false,
                    })
                    .collect(),
            }
        })
        .collect();

    let n_metrics = scenario.assertions.len();
    let metrics = (0..n_metrics)
        .map(|mi| {
            let values: Vec<f64> = all_results.iter().map(|r| r[mi].value).collect();
            let n = values.len() as f64;
            let mean = values.iter().copied().sum::<f64>() / n;
            let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
            let std_dev = variance.sqrt();
            let min = values.iter().cloned().fold(f64::INFINITY, f64::min);
            let max = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let threshold = scenario.assertions[mi].max;
            let pass_rate =
                values.iter().filter(|&&v| v <= threshold).count() as f64 / n;
            MetricStats {
                name: scenario.assertions[mi].metric.to_string(),
                threshold,
                mean,
                std_dev,
                min,
                max,
                pass_rate,
            }
        })
        .collect();

    MonteCarloReport {
        scenario_name: scenario.name.clone(),
        runs: cfg.runs,
        metrics,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_control::cascade::make_cascade;
    use drone_model::vehicle::quadrotor::QuadrotorModel;

    const HOVER_SCENARIO: &str = r#"
        name = "mc_hover"
        duration_s = 8.0
        dt_s = 0.005

        [target]
        z = 5.0

        [initial]
        position = [0.0, 0.0, 0.0]

        [[assertions]]
        metric = "overshoot_percent"
        max = 25.0

        [[assertions]]
        metric = "settling_time_s"
        max = 7.0
    "#;

    #[test]
    fn monte_carlo_hover_10_runs_mostly_pass() {
        let scenario: Scenario = HOVER_SCENARIO.parse().expect("bad TOML");
        let model = QuadrotorModel::mini3();
        let factory: ControllerFactory = Box::new(|m: &dyn VehicleModel| {
            Ok(Box::new(make_cascade(m)) as Box<dyn drone_control::controller::Controller>)
        });
        let cfg = MonteCarloConfig {
            runs: 10,
            pos_noise_m: 0.3,
            vel_noise_ms: 0.1,
            base_seed: 42,
        };

        let report = run_monte_carlo(&scenario, &model, &factory, &cfg);
        report.print();

        // Overshoot metric should mostly pass (≥70% of runs).
        let overshoot = &report.metrics[0];
        assert!(
            overshoot.pass_rate >= 0.7,
            "Overshoot pass rate = {:.0}%, expected >= 70%",
            overshoot.pass_rate * 100.0
        );
    }
}
