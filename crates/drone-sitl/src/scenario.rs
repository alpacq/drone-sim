use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct Scenario {
    pub name: String,
    pub description: Option<String>,

    pub duration_s: f64,

    pub dt_s: f64,

    pub initial: InitialConditions,

    pub target: Target,

    #[serde(default)]
    pub disturbances: Vec<Disturbance>,

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
pub struct Target {
    pub altitude_z: f64,
}

#[derive(Debug, Deserialize)]
pub struct Disturbance {
    pub at_s: f64,

    pub force: [f64; 3],
}

#[derive(Debug, Deserialize)]
pub struct Assertion {
    pub metric: MetricKind,
    pub max: f64,
}

#[derive(Debug, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum MetricKind {
    PositionRmsZ,
    PositionMaxErrorZ,
    OvershootPercent,
    SettlingTimeS,
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
