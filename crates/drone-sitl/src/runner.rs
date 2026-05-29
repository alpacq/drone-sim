use anyhow::Result;
use drone_control::controller::Controller;
use drone_control::target::FlightTarget;
use drone_control::trajectory::Trajectory;
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
pub type ControllerFactory =
    Box<dyn Fn(&dyn VehicleModel) -> Result<Box<dyn Controller>> + Send + Sync>;

/// Run a SITL scenario with the controller produced by `factory`.
///
/// If the scenario defines a trajectory, it is used automatically (overriding
/// the static `[target]` section).  Otherwise the static target is used.
pub fn run_scenario(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
) -> Result<ScenarioReport> {
    // If a trajectory is defined, delegate to the trajectory-aware runner.
    if let Some(traj_def) = &scenario.trajectory {
        let traj = traj_def.clone().into_trajectory();
        return run_scenario_with_trajectory(scenario, model, factory, traj.as_ref());
    }

    let (initial_state, sim_config, disturbances) = prepare_scenario(scenario)?;
    let mut controller = factory(model)?;
    let flight_target = scenario_to_flight_target(&scenario.target);

    let frames = run_with_disturbances(
        initial_state,
        model,
        &sim_config,
        controller.as_mut(),
        &disturbances,
        &flight_target,
    );

    Ok(evaluate_assertions(scenario, frames, &flight_target))
}

/// Run a scenario using a time-varying trajectory instead of the static target.
///
/// The trajectory's `target(time_s)` is called each simulation step.
pub fn run_scenario_with_trajectory(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
    trajectory: &dyn Trajectory,
) -> Result<ScenarioReport> {
    let (initial_state, sim_config, disturbances) = prepare_scenario(scenario)?;
    let mut controller = factory(model)?;

    let frames = run_with_disturbances_traj(
        initial_state,
        model,
        &sim_config,
        controller.as_mut(),
        &disturbances,
        trajectory,
    );

    // Use the terminal trajectory target for assertion evaluation.
    let terminal_target = trajectory.target(scenario.duration_s);
    Ok(evaluate_assertions(scenario, frames, &terminal_target))
}

/// Shared preparation logic: initial state, sim config, disturbances.
#[allow(clippy::type_complexity)]
fn prepare_scenario(
    scenario: &Scenario,
) -> Result<(DroneState, SimConfig, Vec<Box<dyn Disturbance>>)> {
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

    Ok((initial_state, sim_config, disturbances))
}

/// Evaluate scenario assertions against a set of frames and a target.
fn evaluate_assertions(
    scenario: &Scenario,
    frames: Vec<SimFrame>,
    flight_target: &FlightTarget,
) -> ScenarioReport {
    let assertion_results: Vec<AssertionResult> = scenario
        .assertions
        .iter()
        .map(|assertion| {
            let value = compute(&assertion.metric, &frames, flight_target);
            let passed = value <= assertion.max;
            AssertionResult {
                metric: assertion.metric.to_string(),
                value,
                max: assertion.max,
                passed,
            }
        })
        .collect();

    let passed = assertion_results.iter().all(|r| r.passed);
    let frame_count = frames.len();

    ScenarioReport {
        name: scenario.name.clone(),
        passed,
        duration_s: scenario.duration_s,
        frame_count,
        assertions: assertion_results,
        frames,
    }
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

/// Trajectory-aware simulation loop.
///
/// Like [`run_with_disturbances`] but calls `trajectory.target(time)` each step
/// instead of using a fixed target.
pub(crate) fn run_with_disturbances_traj(
    initial_state: DroneState,
    model: &dyn VehicleModel,
    config: &SimConfig,
    controller: &mut dyn Controller,
    disturbances: &[Box<dyn Disturbance>],
    trajectory: &dyn Trajectory,
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

        let target = trajectory.target(time);
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

