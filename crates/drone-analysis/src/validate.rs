use crate::{align::align, error::AnalysisError, report::ValidationReport};
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use drone_sim::{
    integrator::RK4,
    runner::{SimConfig, run},
};
use drone_telemetry::normalize::FlightTrajectory;
use nalgebra::{UnitQuaternion, Vector3};

/// Configuration for [`validate_model`].
///
/// Use [`ValidateConfig::default()`] for sensible defaults, then override
/// individual fields as needed.
#[derive(Debug, Clone)]
pub struct ValidateConfig {
    /// Integration time step for the simulation.
    pub dt: TimeStep,
    /// Position error threshold used to determine "model valid until t":
    /// the last time at which |pos_error| < threshold.
    pub valid_position_threshold_m: f64,
}

impl Default for ValidateConfig {
    fn default() -> Self {
        Self {
            dt: TimeStep::constant(0.02), // 50 Hz — typical telemetry rate
            valid_position_threshold_m: crate::report::VALID_POSITION_THRESHOLD_M,
        }
    }
}

/// Run an open-loop simulation against telemetry and produce a [`ValidationReport`].
///
/// The simulation uses the vehicle's `equilibrium_input` as a constant
/// open-loop input and integrates with RK4.  This is intentional: we are
/// testing the *physical model*, not a controller.
pub fn validate_model(
    model: &dyn VehicleModel,
    telemetry: &FlightTrajectory,
    config: ValidateConfig,
    source_file: String,
) -> Result<ValidationReport, AnalysisError> {
    if telemetry.is_empty() {
        return Err(AnalysisError::EmptyTrajectory);
    }

    let first = &telemetry.points[0];
    let initial_velocity = first.velocity.unwrap_or_else(Vector3::zeros);

    let initial_state = DroneState {
        position: first.position,
        velocity: initial_velocity,
        angular_velocity: Vector3::zeros(),
        orientation: UnitQuaternion::identity(),
        actuator_state: None,
    };

    let sim_config = SimConfig {
        dt: config.dt,
        duration: telemetry.duration_s,
    };

    let equilibrium = model.equilibrium_input();
    let sim_frames = run(initial_state, model, &sim_config, &RK4, |_, _dt| {
        equilibrium.clone()
    });

    let aligned = align(&sim_frames, telemetry);
    let report = ValidationReport::from_aligned(
        aligned,
        source_file,
        config.valid_position_threshold_m,
    );

    Ok(report)
}
