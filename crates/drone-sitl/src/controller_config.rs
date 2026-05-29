//! Controller configuration types and factory builders.
//!
//! Each [`ControllerConfig`] variant fully describes one regulator and can be:
//!
//! * Constructed in code with `ControllerConfig::default()` (uses sensible
//!   tuned defaults for each controller type).
//! * Loaded from a TOML file with [`ControllerConfig::from_file`].
//! * Turned into a ready-to-use [`ControllerFactory`] with
//!   [`ControllerConfig::into_factory`].
//!
//! # TOML format examples
//!
//! ```toml
//! # cascade.toml
//! type = "cascade"
//! max_tilt_deg = 8.6
//! [vel_z]   kp = 0.3  ki = 0.1  kd = 0.0  integral_limit = 0.45  output_limit = 0.45
//! [vel_xy]  kp = 0.4  ki = 0.05 kd = 0.0  integral_limit = 0.5   output_limit = 0.35
//! [att]     kp = 4.0  ki = 0.0  kd = 0.2  integral_limit = 1.0   output_limit = 1.0
//! [att_yaw] kp = 2.0  ki = 0.1  kd = 0.0  integral_limit = 0.5   output_limit = 0.5
//!
//! # lqr.toml
//! type = "lqr"
//! trim_z_m = 5.0
//! q_weights = [1.0, 1.0, 50.0, 0.5, 0.5, 5.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0]
//! r_weights = [0.01, 0.01, 0.01, 0.01]
//! ```

use crate::runner::ControllerFactory;
use drone_model::{
    state::DroneState,
    vehicle::{KnownActuatorInput, VehicleModel},
};
use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;
use std::path::Path;

// ── Top-level enum ────────────────────────────────────────────────────────────

/// Select and configure a flight controller.
///
/// Deserialised with an internal `type` tag:
/// `type = "cascade"` → [`CascadeConfig`],
/// `type = "lqr"` → [`LqrConfig`],
/// `type = "lqi"` → [`LqiConfig`].
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ControllerConfig {
    Cascade(CascadeConfig),
    Lqr(LqrConfig),
    Lqi(LqiConfig),
    Mpc(MpcConfig),
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self::Cascade(CascadeConfig::default())
    }
}

impl ControllerConfig {
    /// Human-readable name, e.g. for table headers.
    pub fn name(&self) -> &str {
        match self {
            Self::Cascade(_) => "Cascade-PID",
            Self::Lqr(_) => "LQR",
            Self::Lqi(_) => "LQI",
            Self::Mpc(_) => "MPC",
        }
    }

    /// Load a config from a TOML file.
    pub fn from_file(path: &Path) -> anyhow::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        Ok(toml::from_str(&content)?)
    }

    /// Convert this config into a [`ControllerFactory`] closure.
    ///
    /// The factory is called once per scenario run; it receives the vehicle
    /// model (needed for equilibrium input / linearisation) and returns a
    /// fresh controller instance.
    pub fn into_factory(self) -> ControllerFactory {
        match self {
            Self::Cascade(c) => cascade_factory(c),
            Self::Lqr(c) => lqr_factory(c),
            Self::Lqi(c) => lqi_factory(c),
            Self::Mpc(c) => mpc_factory(c),
        }
    }
}

// ── PID helper ────────────────────────────────────────────────────────────────

/// Parameters for a single PID loop.
#[derive(Debug, Clone, Deserialize)]
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    /// Anti-windup clamp on the integral accumulator.
    pub integral_limit: f64,
    /// Clamp on the total output.
    pub output_limit: f64,
}

// ── Cascade ───────────────────────────────────────────────────────────────────

/// Configuration for the cascade PID flight controller.
///
/// The cascade has three levels:
/// position → velocity (outer), velocity → attitude (middle),
/// attitude → motor commands (inner).
#[derive(Debug, Clone, Deserialize)]
pub struct CascadeConfig {
    /// Maximum horizontal tilt allowed for XY motion [deg].
    /// Default 8.6° prevents motor saturation when roll and pitch combine.
    pub max_tilt_deg: f64,
    /// Altitude velocity loop: vz error → throttle delta.
    pub vel_z: PidConfig,
    /// Horizontal velocity loops: vx/vy error → target pitch/roll.
    /// The same config is used for both X and Y axes.
    pub vel_xy: PidConfig,
    /// Attitude loops: roll/pitch error → motor command delta.
    /// The same config is used for both roll and pitch.
    pub att: PidConfig,
    /// Yaw attitude loop: yaw error → motor command delta.
    pub att_yaw: PidConfig,
}

impl Default for CascadeConfig {
    fn default() -> Self {
        Self {
            max_tilt_deg: 8.6,
            vel_z: PidConfig { kp: 0.3, ki: 0.1, kd: 0.0, integral_limit: 0.45, output_limit: 0.45 },
            vel_xy: PidConfig { kp: 0.4, ki: 0.05, kd: 0.0, integral_limit: 0.5, output_limit: 0.35 },
            att: PidConfig { kp: 4.0, ki: 0.0, kd: 0.2, integral_limit: 1.0, output_limit: 1.0 },
            att_yaw: PidConfig { kp: 2.0, ki: 0.1, kd: 0.0, integral_limit: 0.5, output_limit: 0.5 },
        }
    }
}

fn cascade_factory(cfg: CascadeConfig) -> ControllerFactory {
    use drone_control::{
        cascade::CascadeController,
        inner_loop::pid_loop::PidLoop,
        mixer::{fixed_wing::FixedWingMixer, quadrotor::QuadrotorMixer},
        profiler::sqrt::SqrtProfiler,
    };

    Box::new(move |m: &dyn VehicleModel| {
        let mixer: Box<dyn drone_control::mixer::Mixer> = match m.equilibrium_input() {
            KnownActuatorInput::Quadrotor(_) => {
                Box::new(QuadrotorMixer::from_equilibrium(m.equilibrium_input()))
            }
            KnownActuatorInput::FixedWing { .. } => {
                Box::new(FixedWingMixer::from_equilibrium(m.equilibrium_input()))
            }
        };

        let p = |c: &PidConfig| PidLoop::new(c.kp, c.ki, c.kd, c.integral_limit, c.output_limit);

        let mut ctrl = CascadeController::new(
            mixer,
            SqrtProfiler::for_altitude(),
            SqrtProfiler::for_horizontal(),
            p(&cfg.vel_z),
            p(&cfg.vel_xy),
            p(&cfg.vel_xy),  // same config for vel_x and vel_y
            p(&cfg.att),
            p(&cfg.att),     // same config for roll and pitch
            p(&cfg.att_yaw),
        );
        ctrl.max_tilt_rad = cfg.max_tilt_deg.to_radians();

        Ok(Box::new(ctrl) as Box<dyn drone_control::controller::Controller>)
    })
}

// ── LQR ───────────────────────────────────────────────────────────────────────

/// Configuration for the Linear-Quadratic Regulator.
///
/// The CARE is solved once per scenario run around a fixed trim point.
/// LQR stabilises that trim; it does **not** track arbitrary setpoints.
/// For tracking use [`LqiConfig`] instead.
///
/// Only quadrotor vehicles are supported.
#[derive(Debug, Clone, Deserialize)]
pub struct LqrConfig {
    /// Trim altitude used for linearisation [m].
    pub trim_z_m: f64,
    /// Q weight vector — 13 elements for a quadrotor:
    /// `[x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]`.
    /// Omit to use the built-in defaults.
    pub q_weights: Option<Vec<f64>>,
    /// R weight vector — 4 elements (one per motor).
    /// Larger values → smoother / less aggressive control effort.
    /// Omit to use the built-in defaults.
    pub r_weights: Option<Vec<f64>>,
}

impl Default for LqrConfig {
    fn default() -> Self {
        Self { trim_z_m: 5.0, q_weights: None, r_weights: None }
    }
}

impl LqrConfig {
    fn q() -> Vec<f64> {
        vec![3.0, 3.0, 80.0, 0.5, 0.5, 10.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0]
    }
    fn r() -> Vec<f64> {
        vec![0.01; 4]
    }
}

fn lqr_factory(cfg: LqrConfig) -> ControllerFactory {
    use drone_control::lqr::LqrController;

    Box::new(move |m: &dyn VehicleModel| {
        let trim_state = quadrotor_trim_state(cfg.trim_z_m);

        let q = cfg.q_weights.clone().unwrap_or_else(LqrConfig::q);
        let r = cfg.r_weights.clone().unwrap_or_else(LqrConfig::r);
        let u_limits = quadrotor_u_limits(m)?;

        LqrController::design(m, &trim_state, &q, &r, u_limits)
            .map(|c| Box::new(c) as Box<dyn drone_control::controller::Controller>)
            .map_err(|e| anyhow::anyhow!("LQR design failed: {}", e))
    })
}

// ── LQI ───────────────────────────────────────────────────────────────────────

/// Configuration for the Linear-Quadratic Integral controller.
///
/// Extends LQR with four integral states `[ξ_x, ξ_y, ξ_z, ξ_ψ]` to
/// eliminate steady-state tracking error under constant disturbances.
///
/// Only quadrotor vehicles are supported.
#[derive(Debug, Clone, Deserialize)]
pub struct LqiConfig {
    /// Trim altitude used for linearisation [m].
    pub trim_z_m: f64,
    /// Q weight vector — 17 elements:
    /// 13 plant states + 4 integral states `[ξ_x, ξ_y, ξ_z, ξ_ψ]`.
    /// Omit to use the built-in defaults.
    pub q_weights: Option<Vec<f64>>,
    /// R weight vector — 4 elements (one per motor).
    /// Omit to use the built-in defaults.
    pub r_weights: Option<Vec<f64>>,
    /// Anti-windup clamp `[m·s, m·s, m·s, rad·s]` for the four integrals.
    /// Omit to use the built-in defaults `[30, 30, 30, 2π]`.
    pub xi_limits: Option<[f64; 4]>,
}

impl Default for LqiConfig {
    fn default() -> Self {
        Self { trim_z_m: 5.0, q_weights: None, r_weights: None, xi_limits: None }
    }
}

impl LqiConfig {
    fn q() -> Vec<f64> {
        vec![
            // 13 plant weights: xyz  vxyz  ωxyz  quaternion
            1.0, 1.0, 100.0,  0.5, 0.5, 12.0,  2.0, 2.0, 2.0,  20.0, 20.0, 20.0, 20.0,
            // 4 integral weights: ξ_x  ξ_y  ξ_z  ξ_ψ
            // Low R (0.005) makes all CARE gains ~40% larger, speeding up
            // the initial climb and recovery from disturbances.  Moderate
            // ξ_z = 6 corrects motor-lag offset without excessive windup.
            5.0, 5.0, 6.0, 2.0,
        ]
    }
    fn r() -> Vec<f64> {
        vec![0.005; 4]
    }
}

fn lqi_factory(cfg: LqiConfig) -> ControllerFactory {
    use drone_control::lqr::{LqiController, quadrotor_c_integral};

    Box::new(move |m: &dyn VehicleModel| {
        let trim_state = quadrotor_trim_state(cfg.trim_z_m);

        let q = cfg.q_weights.clone().unwrap_or_else(LqiConfig::q);
        let r = cfg.r_weights.clone().unwrap_or_else(LqiConfig::r);
        let u_limits = quadrotor_u_limits(m)?;
        let c_int = quadrotor_c_integral(13);

        let mut ctrl = LqiController::design(m, &trim_state, c_int, &q, &r, u_limits)
            .map_err(|e| anyhow::anyhow!("LQI design failed: {}", e))?;

        if let Some(lims) = cfg.xi_limits {
            ctrl.xi_limits = lims;
        }

        Ok(Box::new(ctrl) as Box<dyn drone_control::controller::Controller>)
    })
}

// ── MPC ───────────────────────────────────────────────────────────────────────

/// Configuration for the Model Predictive Controller.
///
/// At each simulation step the MPC re-linearises the model around the current
/// state, builds a condensed finite-horizon QP, and solves it with projected
/// gradient descent.
///
/// Only quadrotor vehicles are supported.
#[derive(Debug, Clone, Deserialize)]
pub struct MpcConfig {
    /// Prediction (= control) horizon in steps.
    pub horizon: usize,
    /// Prediction step size [s]. Deliberately coarser than the simulation `dt_s`.
    pub dt_s: f64,
    /// Q weight vector — 13 elements:
    /// `[x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]`.
    /// Omit to use the built-in defaults.
    pub q_weights: Option<Vec<f64>>,
    /// R weight vector — 4 elements (one per motor).
    /// Omit to use the built-in defaults.
    pub r_weights: Option<Vec<f64>>,
    /// Integral cost weights — 3 elements `[ξ_x, ξ_y, ξ_z]`.
    /// Higher values → faster steady-state correction.
    /// Omit to use the built-in defaults.
    pub qi_weights: Option<Vec<f64>>,
    /// Anti-windup clamp for integrals `[m·s, m·s, m·s]`.
    /// Omit to use the built-in defaults.
    pub xi_limits: Option<Vec<f64>>,
}

impl Default for MpcConfig {
    fn default() -> Self {
        // dt_s is the MPC's internal prediction step, deliberately coarser than
        // the simulation step.  With horizon=10 and dt_s=0.5 the prediction
        // window spans 5 s — long enough to cover a typical settling time and
        // allow the solver to plan a meaningful climb trajectory.
        Self { horizon: 10, dt_s: 0.5, q_weights: None, r_weights: None, qi_weights: None, xi_limits: None }
    }
}

impl MpcConfig {
    fn q() -> Vec<f64> {
        vec![15.0, 15.0, 50.0, 2.0, 2.0, 8.0, 4.0, 4.0, 4.0, 15.0, 15.0, 15.0, 15.0]
    }
    fn r() -> Vec<f64> {
        vec![0.01; 4]
    }
    fn qi() -> Vec<f64> {
        vec![1.0, 1.0, 3.0]
    }
    fn xi_lim() -> Vec<f64> {
        vec![3.0, 3.0, 3.0]
    }
}

fn mpc_factory(cfg: MpcConfig) -> ControllerFactory {
    use drone_control::MpcController;
    use std::sync::Arc;

    Box::new(move |m: &dyn VehicleModel| {
        let hover_w = match m.equilibrium_input() {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => anyhow::bail!("MPC is only supported for quadrotor vehicles"),
        };
        let u_limits = vec![(0.0, hover_w * 2.0); 4];

        let model: Arc<dyn VehicleModel> = Arc::from(m.clone_box());
        let q = cfg.q_weights.clone().unwrap_or_else(MpcConfig::q);
        let r = cfg.r_weights.clone().unwrap_or_else(MpcConfig::r);
        let qi_vec = cfg.qi_weights.clone().unwrap_or_else(MpcConfig::qi);
        let xi_vec = cfg.xi_limits.clone().unwrap_or_else(MpcConfig::xi_lim);
        assert_eq!(qi_vec.len(), 3, "qi_weights must have 3 entries");
        assert_eq!(xi_vec.len(), 3, "xi_limits must have 3 entries");
        let qi: [f64; 3] = [qi_vec[0], qi_vec[1], qi_vec[2]];
        let xi_lim: [f64; 3] = [xi_vec[0], xi_vec[1], xi_vec[2]];

        Ok(Box::new(MpcController::new(model, cfg.horizon, cfg.dt_s, q, r, qi, xi_lim, u_limits))
            as Box<dyn drone_control::controller::Controller>)
    })
}

// ── Shared helpers ────────────────────────────────────────────────────────────

fn quadrotor_trim_state(trim_z_m: f64) -> DroneState {
    DroneState {
        position: Vector3::new(0.0, 0.0, trim_z_m),
        velocity: Vector3::zeros(),
        orientation: UnitQuaternion::identity(),
        angular_velocity: Vector3::zeros(),
        actuator_state: None,
    }
}

fn quadrotor_u_limits(m: &dyn VehicleModel) -> anyhow::Result<Vec<(f64, f64)>> {
    let hover_w = match m.equilibrium_input() {
        KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
        _ => anyhow::bail!("LQR / LQI are only supported for quadrotor vehicles"),
    };
    Ok(vec![(0.0, hover_w * 2.0); 4])
}
