use crate::disturbance::DisturbanceConfig;
use drone_control::target::FlightTarget;
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

#[derive(Debug, Clone, Deserialize)]
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

    /// Optional time-varying trajectory. When present, overrides the static `[target]`.
    #[serde(default)]
    pub trajectory: Option<ScenarioTrajectoryDef>,

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


#[derive(Debug, Clone, Deserialize)]
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

#[derive(Debug, Clone, Deserialize)]
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

#[derive(Debug, Clone, Deserialize, PartialEq)]
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

/// Trajectory definition that can be deserialized from a scenario TOML file.
///
/// The `type` field selects the variant:
/// - `"hold"` — static hold at a single position.
/// - `"waypoint"` — piecewise-linear path through timed waypoints.
/// - `"circle"` — horizontal circular orbit at fixed altitude.
#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ScenarioTrajectoryDef {
    /// Hold a fixed position.
    Hold {
        z: f64,
        #[serde(default)]
        x: Option<f64>,
        #[serde(default)]
        y: Option<f64>,
        #[serde(default)]
        yaw: Option<f64>,
    },
    /// Piecewise-linear path through timed waypoints.
    Waypoint {
        waypoints: Vec<WaypointEntry>,
    },
    /// Horizontal circular orbit.
    Circle {
        cx: f64,
        cy: f64,
        radius: f64,
        omega_deg_s: f64,
        altitude_m: f64,
    },
}

/// A single waypoint in a [`ScenarioTrajectoryDef::Waypoint`] trajectory.
#[derive(Debug, Deserialize, Clone)]
pub struct WaypointEntry {
    /// Time at which this waypoint should be reached [s].
    pub time_s: f64,
    /// Altitude [m] — always required.
    pub z: f64,
    /// X position [m] — optional.
    #[serde(default)]
    pub x: Option<f64>,
    /// Y position [m] — optional.
    #[serde(default)]
    pub y: Option<f64>,
    /// Yaw angle [rad] — optional.
    #[serde(default)]
    pub yaw: Option<f64>,
}

impl ScenarioTrajectoryDef {
    /// Convert to a heap-allocated [`Trajectory`](drone_control::Trajectory).
    pub fn into_trajectory(self) -> Box<dyn drone_control::trajectory::Trajectory> {
        match self {
            Self::Hold { x, y, z, yaw } => Box::new(drone_control::HoldTrajectory {
                inner: FlightTarget {
                    x,
                    y,
                    z: Some(z),
                    yaw,
                },
            }),
            Self::Waypoint { waypoints } => {
                let wps = waypoints
                    .into_iter()
                    .map(|w| {
                        (
                            w.time_s,
                            FlightTarget {
                                x: w.x,
                                y: w.y,
                                z: Some(w.z),
                                yaw: w.yaw,
                            },
                        )
                    })
                    .collect();
                Box::new(drone_control::WaypointTrajectory::new(wps))
            }
            Self::Circle {
                cx,
                cy,
                radius,
                omega_deg_s,
                altitude_m,
            } => Box::new(drone_control::CircleTrajectory {
                cx,
                cy,
                radius,
                omega: omega_deg_s.to_radians(),
                altitude_m,
            }),
        }
    }
}
