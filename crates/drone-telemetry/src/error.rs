use thiserror::Error;

/// Errors produced by the telemetry parsing and normalisation pipeline.
///
/// Using a typed enum (rather than `anyhow::Error`) lets callers distinguish
/// between a missing file, a malformed SRT stream, and insufficient GPS
/// coverage — each requiring a different user-facing message or recovery path.
#[derive(Debug, Error)]
pub enum TelemetryError {
    /// The source file could not be read from disk.
    #[error("cannot read '{path}': {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },

    /// The SRT content contained no valid telemetry blocks.
    #[error("no telemetry frames found — the SRT file may be empty or unparseable")]
    Empty,

    /// Normalisation requires at least one frame with GPS coordinates, but none
    /// were found.  Check that Video Captions was enabled during the flight.
    #[error("no GPS frames in telemetry — is Video Captions enabled?")]
    NoGpsFrames,

    /// Velocity estimation uses central differences and requires ≥ 3 GPS frames.
    #[error("not enough GPS frames ({found}) — need at least 3 to compute velocity")]
    NotEnoughGpsFrames { found: usize },
}
