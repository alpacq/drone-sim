use anyhow::Result;
use drone_control::controller::Controller;
use drone_control::target::FlightTarget;
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use drone_sim::runner::{SimConfig, SimFrame};
use nalgebra::{UnitQuaternion, Vector3};

use crate::disturbance::Disturbance;
use crate::metrics::compute;
use crate::report::{AssertionResult, ScenarioReport};
use crate::scenario::Scenario;

/// Creates a fresh controller for a given vehicle model.
/// Using a factory (rather than a pre-built instance) guarantees each
/// simulation run starts from a clean controller state.
pub type ControllerFactory = Box<dyn Fn(&dyn VehicleModel) -> Result<Box<dyn Controller>>>;

/// Run a SITL scenario with the controller produced by `factory`.
/// Passing the factory instead of constructing the controller internally
/// decouples the runner from any specific controller implementation.
pub fn run_scenario(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
) -> Result<ScenarioReport> {
    let dt = TimeStep::new(scenario.dt_s).map_err(|e| anyhow::anyhow!("Invalid dt: {}", e))?;

    let [roll_deg, pitch_deg, yaw_deg] = scenario.initial.attitude_deg;
    let initial_orientation = UnitQuaternion::from_euler_angles(
        roll_deg.to_radians(),
        pitch_deg.to_radians(),
        yaw_deg.to_radians(),
    );

    let initial_state = DroneState {
        position: Vector3::from(scenario.initial.position),
        velocity: Vector3::from(scenario.initial.velocity),
        angular_velocity: Vector3::zeros(),
        orientation: initial_orientation,
        actuator_state: None,
    };

    let mut controller = factory(model)?;

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

    let flight_target = scenario_to_flight_target(&scenario.target);

    let frames = run_with_disturbances(
        initial_state,
        model,
        &sim_config,
        controller.as_mut(),
        &disturbances,
        &flight_target,
    );

    let assertion_results: Vec<AssertionResult> = scenario
        .assertions
        .iter()
        .map(|assertion| {
            let value = compute(&assertion.metric, &frames, &flight_target);
            let passed = value <= assertion.max;
            AssertionResult {
                metric: assertion.metric.to_string(), // Display, not Debug
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

/// Canonical simulation loop shared by scenario testing and controller comparison.
///
/// Applies disturbances, calls the controller, steps actuator dynamics,
/// then integrates with RK4. Returns the full frame history.
pub(crate) fn run_with_disturbances(
    initial_state: DroneState,
    model: &dyn VehicleModel,
    config: &SimConfig,
    controller: &mut dyn Controller,
    disturbances: &[Box<dyn Disturbance>],
    target: &FlightTarget,
) -> Vec<SimFrame> {
    use drone_sim::integrator::{Integrator as _, RK4};

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

        let input = controller.update(&state, target, config.dt);

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

/// Convert a scenario target to a `FlightTarget`.
/// Lives in the runner (adapter layer) so `ScenarioTarget` stays
/// a pure data struct with no dependency on the control library.
pub(crate) fn scenario_to_flight_target(
    t: &crate::scenario::ScenarioTarget,
) -> FlightTarget {
    // `t.z` is a required TOML field → always `Some`.
    // `t.x` and `t.y` are optional → `None` means "do not control that axis".
    FlightTarget {
        x:   t.x,
        y:   t.y,
        z:   Some(t.z),
        yaw: t.yaw,
    }
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

        [target]
        z = 5.0

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
        use drone_control::cascade::make_cascade;
        let model = QuadrotorModel::mini3();
        let scenario = HOVER_SCENARIO
            .parse::<crate::scenario::Scenario>()
            .expect("Incorrect TOML");
        let cascade: ControllerFactory = Box::new(|m| Ok(Box::new(make_cascade(m))));
        let report = run_scenario(&scenario, &model, &cascade).expect("Simulation error");
        report.print();
        assert!(report.passed, "Scenario hover didn't pass assertion");
    }
}

