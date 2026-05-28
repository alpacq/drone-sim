use crate::disturbance::DisturbanceConfig;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct Scenario {
    pub name: String,
    pub description: Option<String>,

    pub duration_s: f64,

    pub dt_s: f64,

    pub initial: InitialConditions,

    pub target_z: f64,

    #[serde(default)]
    pub disturbances: Vec<DisturbanceConfig>,

    pub assertions: Vec<Assertion>,
}

#[derive(Debug, Deserialize)]
pub struct InitialConditions {
    #[serde(default)]
    pub position: [f64; 3],

    #[serde(default)]
    pub velocity: [f64; 3],
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

#[derive(Debug, Deserialize, Clone)]
pub struct Target3d {
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub z: Option<f64>,
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

impl Scenario {
    pub fn from_file(path: &std::path::Path) -> anyhow::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        let scenario = toml::from_str(&content)?;
        Ok(scenario)
    }

    pub fn from_str(s: &str) -> anyhow::Result<Self> {
        Ok(toml::from_str(s)?)
    }
}
