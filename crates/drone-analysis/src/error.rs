use thiserror::Error;

/// Errors returned by the model-validation pipeline.
#[derive(Debug, Error)]
pub enum AnalysisError {
    /// The telemetry trajectory passed to `validate_model` contained no points.
    /// Validation requires at least one telemetry sample to compare against.
    #[error("telemetry trajectory is empty — validation requires at least one point")]
    EmptyTrajectory,
}
