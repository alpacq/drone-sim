use anyhow::Result;
use drone_control::controller::{Controller, cascade::make_cascade};
use drone_control::target::FlightTarget;
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use drone_sim::{
    integrator::Integrator,
    runner::{SimConfig, SimFrame},
};
use nalgebra::{UnitQuaternion, Vector3};

use crate::disturbance::Disturbance;
use crate::metrics::compute;
use crate::report::{AssertionResult, ScenarioReport};
use crate::scenario::Scenario;

pub fn run_scenario(scenario: &Scenario, model: &dyn VehicleModel) -> Result<ScenarioReport> {
    let dt = TimeStep::new(scenario.dt_s).map_err(|e| anyhow::anyhow!("Invalid dt: {}", e))?;

    let initial_state = DroneState {
        position: Vector3::from(scenario.initial.position),
        velocity: Vector3::from(scenario.initial.velocity),
        angular_velocity: Vector3::zeros(),
        orientation: UnitQuaternion::identity(),
        actuator_state: None,
    };

    let mut controller = make_cascade(model);

    let disturbances: Vec<Box<dyn Disturbance>> = scenario
        .disturbances
        .iter()
        .cloned()
        .map(|d| d.into_disturbance())
        .collect();

    let sim_config = SimConfig {
        dt,
        duration: scenario.duration_s,
    };

    let frames = run_with_disturbances(
        initial_state,
        model,
        &sim_config,
        &mut controller,
        &disturbances,
        scenario.target_z,
    );

    let assertion_results: Vec<AssertionResult> = scenario
        .assertions
        .iter()
        .map(|assertion| {
            let value = compute(
                &assertion.metric,
                &frames,
                &FlightTarget::altitude(scenario.target_z),
            );
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
    model: &dyn VehicleModel,
    config: &SimConfig,
    controller: &mut dyn Controller,
    disturbances: &[Box<dyn Disturbance>],
    target_z: f64,
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
        for disturbance in disturbances {
            if disturbance.is_active(time) {
                disturbance.apply(&mut state, model, config.dt);
            }
        }

        let target = FlightTarget::altitude(target_z);
        let input = controller.update(&state, &target, config.dt);

        model.step_actuators(&mut state, &input, config.dt);

        state = RK4.step(model, &state, &input, config.dt);
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
    use drone_model::vehicle::quadrotor::QuadrotorModel;

    // NOTE: position_rms_z is intentionally NOT asserted here.
    // This is a lift-off test (z=0 → 5m): the RMS is dominated by the initial
    // 5m error and cannot be meaningfully bounded. Use wind_rejection scenario
    // (steady-state hover) to test RMS and max error.
    const HOVER_SCENARIO: &str = r#"
        name = "test_hover"
        duration_s = 8.0
        dt_s = 0.005

        target_z = 5.0

        [initial]
        position = [0.0, 0.0, 0.0]

        [[assertions]]
        metric = "overshoot_percent"
        max = 20.0

        [[assertions]]
        metric = "settling_time_s"
        max = 6.0
    "#;

    #[test]
    fn hover_scenario_passes() {
        let model = QuadrotorModel::mini3();
        let scenario = crate::scenario::Scenario::from_str(HOVER_SCENARIO).expect("Incorrect TOML");
        let report = run_scenario(&scenario, &model).expect("Simulation error");
        report.print();
        assert!(report.passed, "Scenario hover didn't pass assertion");
    }
}
