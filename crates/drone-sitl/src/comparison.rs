use anyhow::Result;
use drone_control::target::FlightTarget;
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use drone_sim::runner::{SimConfig, SimFrame};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{disturbance::Disturbance, metrics, runner::run_with_disturbances, scenario::Scenario};

// Re-exported so existing code that imports from `drone_sitl::comparison` keeps compiling.
pub use crate::runner::ControllerFactory;

#[derive(Debug)]
pub struct ControllerResult {
    pub name: String,
    pub frames: Vec<SimFrame>,
    // ── Metryki ───────────────────────────────────────────────────
    pub rms_error_z: f64,
    pub max_error_z: f64,
    pub overshoot_pct: f64,
    pub settling_time_s: f64,
    pub rise_time_s: f64,
    pub steady_state_err: f64,
    pub control_energy: f64,
    pub max_control_rate: f64,
}

pub struct ComparisonReport {
    pub scenario_name: String,
    pub target_z: f64,
    pub results: Vec<ControllerResult>,
}

impl ComparisonReport {
    pub fn print(&self) {
        println!("\n╔══════════════════════════════════════════════════════════════╗");
        println!("║  Controllers comparison: {}", self.scenario_name);
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!(
            "║ {:<22} {:>8} {:>8} {:>8} {:>8} {:>8}",
            "Controller", "RMS[m]", "OS[%]", "ST[s]", "RT[s]", "Energia"
        );
        println!("╠══════════════════════════════════════════════════════════════╣");

        for r in &self.results {
            println!(
                "║ {:<22} {:>8.3} {:>8.1} {:>8.2} {:>8.2} {:>8.0}",
                r.name,
                r.rms_error_z,
                r.overshoot_pct,
                r.settling_time_s,
                r.rise_time_s,
                r.control_energy,
            );
        }

        println!("╠══════════════════════════════════════════════════════════════╣");

        if let Some(best_rms) = self
            .results
            .iter()
            .min_by(|a, b| a.rms_error_z.partial_cmp(&b.rms_error_z).unwrap())
        {
            println!("║  Best RMS:      {}", best_rms.name);
        }
        if let Some(best_energy) = self
            .results
            .iter()
            .min_by(|a, b| a.control_energy.partial_cmp(&b.control_energy).unwrap())
        {
            println!("║  Best energy: {}", best_energy.name);
        }
        if let Some(best_st) = self
            .results
            .iter()
            .min_by(|a, b| a.settling_time_s.partial_cmp(&b.settling_time_s).unwrap())
        {
            println!("║  Quickest:        {}", best_st.name);
        }
        println!("╚══════════════════════════════════════════════════════════════╝");
    }

    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "controller,rms_z,max_error_z,overshoot_pct,\
             settling_time_s,rise_time_s,steady_state_err,\
             control_energy,max_control_rate\n",
        );
        for r in &self.results {
            out.push_str(&format!(
                "{},{:.4},{:.4},{:.2},{:.3},{:.3},{:.4},{:.1},{:.1}\n",
                r.name,
                r.rms_error_z,
                r.max_error_z,
                r.overshoot_pct,
                r.settling_time_s,
                r.rise_time_s,
                r.steady_state_err,
                r.control_energy,
                r.max_control_rate,
            ));
        }
        out
    }

    pub fn trajectories_to_csv(&self) -> String {
        let mut out = String::from("time");
        for r in &self.results {
            out.push_str(&format!(",z_{},vz_{}", r.name, r.name));
        }
        out.push('\n');

        let max_frames = self
            .results
            .iter()
            .map(|r| r.frames.len())
            .max()
            .unwrap_or(0);

        for i in 0..max_frames {
            if let Some(first) = self.results.first() {
                if i >= first.frames.len() {
                    break;
                }
                out.push_str(&format!("{:.4}", first.frames[i].time));
            }
            for r in &self.results {
                if i < r.frames.len() {
                    out.push_str(&format!(
                        ",{:.4},{:.4}",
                        r.frames[i].state.position.z, r.frames[i].state.velocity.z,
                    ));
                } else {
                    out.push_str(",N/A,N/A");
                }
            }
            out.push('\n');
        }
        out
    }
}

pub fn compare_controllers(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factories: &[(&str, ControllerFactory)],
) -> Result<ComparisonReport> {
    let dt = TimeStep::new(scenario.dt_s).map_err(|e| anyhow::anyhow!("{}", e))?;
    let target_z = scenario.target_z;

    let target = FlightTarget::altitude(target_z);

    let disturbances: Vec<Box<dyn Disturbance>> = scenario
        .disturbances
        .iter()
        .map(|d| d.clone().into_disturbance())
        .collect();

    let mut results = Vec::new();

    let config = SimConfig {
        dt,
        duration: scenario.duration_s,
    };

    for (name, factory) in factories {
        let mut controller = factory(model)
            .map_err(|e| anyhow::anyhow!("Cannot create controller '{}': {}", name, e))?;

        let initial_state = DroneState {
            position: Vector3::from(scenario.initial.position),
            velocity: Vector3::from(scenario.initial.velocity),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        };

        let frames = run_with_disturbances(
            initial_state,
            model,
            &config,
            controller.as_mut(),
            &disturbances,
            target_z,
        );

        let result = ControllerResult {
            name: name.to_string(),
            rms_error_z: metrics::position_rms_z(&frames, &target),
            max_error_z: metrics::position_max_error_z(&frames, &target),
            overshoot_pct: metrics::overshoot_percent(&frames, &target),
            settling_time_s: metrics::settling_time_s(&frames, &target),
            rise_time_s: metrics::rise_time_s(&frames, target_z),
            steady_state_err: metrics::steady_state_error(&frames, target_z),
            control_energy: metrics::control_energy(&frames),
            max_control_rate: metrics::max_control_rate(&frames),
            frames,
        };

        results.push(result);
    }

    Ok(ComparisonReport {
        scenario_name: scenario.name.clone(),
        target_z,
        results,
    })
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::Scenario;
    use drone_control::cascade::make_cascade;
    use drone_model::vehicle::quadrotor::QuadrotorModel;

    const SIMPLE_SCENARIO: &str = r#"
        name = "test_comparison"
        duration_s = 5.0
        dt_s = 0.01

        target_z = 3.0

        [initial]
        position = [0.0, 0.0, 0.0]

        [[assertions]]
        metric = "position_rms_z"
        max = 2.0
    "#;

    #[test]
    fn comparison_of_two_controllers() {
        let model = QuadrotorModel::mini3_simple();
        let scenario = Scenario::from_str(SIMPLE_SCENARIO).unwrap();

        let pid: ControllerFactory = Box::new(|m| Ok(Box::new(make_cascade(m))));
        let pid2: ControllerFactory = Box::new(|m| Ok(Box::new(make_cascade(m))));

        let factories = vec![("PID-1", pid), ("PID-2", pid2)];

        let report = compare_controllers(&scenario, &model, &factories).unwrap();

        assert_eq!(report.results.len(), 2);

        let r0 = &report.results[0];
        let r1 = &report.results[1];
        assert!(
            (r0.rms_error_z - r1.rms_error_z).abs() < 1e-6,
            "Identical controllers should give identical results"
        );
    }
}
