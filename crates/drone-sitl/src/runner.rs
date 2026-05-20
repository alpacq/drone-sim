use anyhow::Result;
use drone_control::cascade::AltitudeController;
use drone_model::{params::DroneParams, state::DroneState, time::TimeStep};
use drone_sim::runner::{SimConfig, SimFrame};
use nalgebra::{UnitQuaternion, Vector3};

use crate::metrics::compute;
use crate::report::{AssertionResult, ScenarioReport};
use crate::scenario::{Scenario, Target};

pub fn run_scenario(scenario: &Scenario, params: &DroneParams) -> Result<ScenarioReport> {
    let dt = TimeStep::new(scenario.dt_s).map_err(|e| anyhow::anyhow!("Invalid dt: {}", e))?;

    let initial_state = DroneState {
        position: Vector3::from(scenario.initial.position),
        velocity: Vector3::from(scenario.initial.velocity),
        angular_velocity: Vector3::zeros(),
        orientation: UnitQuaternion::identity(),
    };

    let mut controller = AltitudeController::new(params);

    let disturbances = &scenario.disturbances;

    let sim_config = SimConfig {
        dt,
        duration: scenario.duration_s,
    };

    let frames = run_with_disturbances(
        initial_state,
        params,
        &sim_config,
        &mut controller,
        disturbances,
        &scenario.target,
    );

    let assertion_results: Vec<AssertionResult> = scenario
        .assertions
        .iter()
        .map(|assertion| {
            let value = compute(&assertion.metric, &frames, &scenario.target);
            let passed = value <= assertion.max;
            AssertionResult {
                metric: format!("{:?}", assertion.metric),
                value,
                max: assertion.max,
                passed,
            }
        })
        .collect();

    let passed = assertion_results.iter().all(|r| r.passed);

    Ok(ScenarioReport {
        name: scenario.name.clone(),
        passed,
        duration_s: scenario.duration_s,
        frame_count: frames.len(),
        assertions: assertion_results,
        frames,
    })
}

fn run_with_disturbances(
    initial_state: DroneState,
    params: &DroneParams,
    config: &SimConfig,
    controller: &mut AltitudeController,
    disturbances: &[crate::scenario::Disturbance],
    target: &Target,
) -> Vec<SimFrame> {
    use drone_sim::integrator::RK4;

    let mut state = initial_state;
    let mut time = 0.0_f64;
    let steps = (config.duration / config.dt.seconds()).ceil() as usize;
    let mut frames = Vec::with_capacity(steps + 1);

    frames.push(SimFrame {
        time,
        state: state.clone(),
    });

    for _ in 0..steps {
        let active_disturbance = disturbances
            .iter()
            .find(|d| (d.at_s - time).abs() < config.dt.seconds() / 2.0);

        if let Some(dist) = active_disturbance {
            let impulse = Vector3::from(dist.force) * config.dt.seconds() / params.mass;
            state.velocity += impulse;
        }

        let input = controller.update(&state, target.altitude_z, config.dt);

        use drone_sim::integrator::Integrator;
        state = RK4.step(&state, &input, params, config.dt);
        time += config.dt.seconds();

        frames.push(SimFrame {
            time,
            state: state.clone(),
        });
    }

    frames
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::params::DroneParams;

    // NOTE: position_rms_z is intentionally NOT asserted here.
    // This is a lift-off test (z=0 → 5m): the RMS is dominated by the initial
    // 5m error and cannot be meaningfully bounded. Use wind_rejection scenario
    // (steady-state hover) to test RMS and max error.
    const HOVER_SCENARIO: &str = r#"
        name = "test_hover"
        duration_s = 8.0
        dt_s = 0.005

        [initial]
        position = [0.0, 0.0, 0.0]

        [target]
        altitude_z = 5.0

        [[assertions]]
        metric = "overshoot_percent"
        max = 20.0

        [[assertions]]
        metric = "settling_time_s"
        max = 6.0
    "#;

    #[test]
    fn hover_scenariusz_przechodzi() {
        let params = DroneParams::mini3();
        let scenario = crate::scenario::Scenario::from_str(HOVER_SCENARIO).expect("Incorrect TOML");
        let report = run_scenario(&scenario, &params).expect("Simulation error");

        println!("\nPierwsze 20 kroków:");
        println!(
            "{:>8} {:>10} {:>10} {:>10} {:>10}",
            "time", "z", "vz", "motor_avg", "error_z"
        );
        for f in report.frames.iter().take(20) {
            let motor_avg = {
                let input = {
                    let mut ctrl = drone_control::cascade::AltitudeController::new(&params);
                    ctrl.update(&f.state, 5.0, drone_model::time::TimeStep::constant(0.005))
                };
                input.motor_speeds.sum() / 4.0
            };
            println!(
                "{:>8.3} {:>10.4} {:>10.4} {:>10.2} {:>10.4}",
                f.time,
                f.state.position.z,
                f.state.velocity.z,
                motor_avg,
                5.0 - f.state.position.z,
            );
        }

        report.print();

        assert!(report.passed, "Scenario hover didn't pass assertion");
    }
}
