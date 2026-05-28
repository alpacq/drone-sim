use crate::disturbance::DisturbanceConfig;
use serde::Deserialize;

/// Which vehicle model to simulate.
///
/// Scenarios that omit this field default to `quadrotor_mini3`.
#[derive(Debug, Deserialize, Default, Clone, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum VehicleKind {
    /// DJI Mini 3 quadrotor (full model with ISA atmosphere and motor dynamics).
    #[default]
    QuadrotorMini3,
    /// DJI Mini 3 quadrotor, simplified (constant density atmosphere, faster linearisation).
    QuadrotorMini3Simple,
    /// F-16A fighter jet (NASA TP-1538 aerodynamic model, F110 engine).
    F16,
}

#[derive(Debug, Deserialize)]
pub struct Scenario {
    pub name: String,
    pub description: Option<String>,

    pub duration_s: f64,

    pub dt_s: f64,

    pub initial: InitialConditions,

    /// Vehicle model to use.  Omit to use the default quadrotor.
    #[serde(default)]
    pub vehicle: VehicleKind,

    /// Flight target for the scenario.
    /// TOML example (altitude-only):
    /// ```toml
    /// [target]
    /// z = 5.0
    /// ```
    /// Full 3-D + yaw:
    /// ```toml
    /// [target]
    /// z = 5.0
    /// x = 1.0
    /// y = 2.0
    /// yaw = 0.5
    /// ```
    pub target: ScenarioTarget,

    #[serde(default)]
    pub disturbances: Vec<DisturbanceConfig>,

    pub assertions: Vec<Assertion>,
}

/// Target setpoint expressed in a scenario file.
///
/// `z` (altitude) is always required; `x`, `y`, and `yaw` are optional
/// (defaulting to 0 / absent).
#[derive(Debug, Deserialize, Clone)]
pub struct ScenarioTarget {
    pub z: f64,
    #[serde(default)]
    pub x: Option<f64>,
    #[serde(default)]
    pub y: Option<f64>,
    #[serde(default)]
    pub yaw: Option<f64>,
}


#[derive(Debug, Deserialize)]
pub struct InitialConditions {
    #[serde(default)]
    pub position: [f64; 3],

    #[serde(default)]
    pub velocity: [f64; 3],

    /// Initial attitude expressed as [roll, pitch, yaw] in degrees.
    ///
    /// Omit (or set to `[0, 0, 0]`) to start with a level (identity)
    /// orientation.  Required for fixed-wing scenarios where the aircraft
    /// must start at a trim angle-of-attack (e.g. F-16 at α = 5°).
    #[serde(default)]
    pub attitude_deg: [f64; 3],
}

#[derive(Debug, Deserialize)]
pub struct Assertion {
    pub metric: MetricKind,
    pub max: f64,
}

#[derive(Debug, Deserialize, PartialEq, Clone)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Debug, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum MetricKind {
    PositionRms3d,
    PositionRmsAxis(Axis),
    PositionMaxError3d,
    PositionMaxErrorAxis(Axis),
    VelocityRms3d,
    VelocityRmsAxis(Axis),
    AttitudeRms,
    AttitudeMaxError,
    OvershootPercent,
    SettlingTimeS,
    RiseTimeS,
    SteadyStateError,
    ControlEnergy,
    MaxControlRate,
}

impl std::fmt::Display for MetricKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PositionRms3d          => write!(f, "PositionRms3d"),
            Self::PositionRmsAxis(a)     => write!(f, "PositionRms{a:?}"),
            Self::PositionMaxError3d     => write!(f, "PositionMaxError3d"),
            Self::PositionMaxErrorAxis(a)=> write!(f, "PositionMaxError{a:?}"),
            Self::VelocityRms3d          => write!(f, "VelocityRms3d"),
            Self::VelocityRmsAxis(a)     => write!(f, "VelocityRms{a:?}"),
            Self::AttitudeRms            => write!(f, "AttitudeRms"),
            Self::AttitudeMaxError       => write!(f, "AttitudeMaxError"),
            Self::OvershootPercent       => write!(f, "OvershootPercent"),
            Self::SettlingTimeS          => write!(f, "SettlingTimeS"),
            Self::RiseTimeS              => write!(f, "RiseTimeS"),
            Self::SteadyStateError       => write!(f, "SteadyStateError"),
            Self::ControlEnergy          => write!(f, "ControlEnergy"),
            Self::MaxControlRate         => write!(f, "MaxControlRate"),
        }
    }
}

/// Errors that can occur while loading a [`Scenario`] from a file or string.
///
/// Using a typed error (rather than `anyhow::Error`) lets callers distinguish
/// I/O failures (permissions, missing file) from TOML syntax errors.
#[derive(Debug, thiserror::Error)]
pub enum ScenarioError {
    #[error("cannot read scenario file: {0}")]
    Io(#[from] std::io::Error),
    #[error("TOML parse error: {0}")]
    Toml(#[from] toml::de::Error),
}

impl Scenario {
    pub fn from_file(path: &std::path::Path) -> Result<Self, ScenarioError> {
        let content = std::fs::read_to_string(path)?;
        Ok(toml::from_str(&content)?)
    }
}

/// Parse a scenario from a TOML string.
///
/// Implements [`std::str::FromStr`] so callers can use `s.parse::<Scenario>()`
/// or the trait method `Scenario::from_str(s)` when the trait is in scope.
impl std::str::FromStr for Scenario {
    type Err = ScenarioError;

    fn from_str(s: &str) -> Result<Self, ScenarioError> {
        Ok(toml::from_str(s)?)
    }
}
