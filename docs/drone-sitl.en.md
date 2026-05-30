# `drone-sitl` — Reference Documentation

## 1. Overview

The `drone-sitl` crate provides a **SITL** (Software-In-The-Loop) testing environment for the drone flight simulator. It supports:

- defining test scenarios in TOML files (flight goal, trajectory, disturbances, assertions),
- configuring and comparing controllers (Cascade-PID, LQR, LQI),
- running simulations with disturbances (wind gusts, turbulence, motor failure),
- computing control quality metrics (RMS, overshoot, settling time, control energy, etc.),
- generating reports and CSV exports,
- Monte Carlo analysis with parallel execution (Rayon).

Public modules:

- `scenario` — test scenario definition
- `controller_config` — controller configuration
- `runner` — SITL simulation loop
- `disturbance` — disturbance models
- `metrics` — metric functions
- `report` — scenario report
- `comparison` — controller comparison
- `monte_carlo` — Monte Carlo analysis

Re-exports from `lib.rs`:

```rust path=null start=null
pub use controller_config::{CascadeConfig, ControllerConfig, LqiConfig, LqrConfig, PidConfig};
pub use monte_carlo::{MonteCarloConfig, MonteCarloReport};
```

---

## 2. Module `scenario`

Responsible for deserialising test scenarios from TOML format.

### `Scenario`

Main struct defining a single SITL scenario.

```rust path=null start=null
pub struct Scenario {
    pub name: String,
    pub description: Option<String>,
    pub duration_s: f64,
    pub dt_s: f64,
    pub initial: InitialConditions,
    pub vehicle: VehicleKind,
    pub target: ScenarioTarget,
    pub trajectory: Option<ScenarioTrajectoryDef>,
    pub disturbances: Vec<DisturbanceConfig>,
    pub assertions: Vec<Assertion>,
}
```

Fields:

- `name` — scenario name (displayed in reports).
- `description` — optional text description.
- `duration_s` — simulation duration [s].
- `dt_s` — simulation time step [s].
- `initial` — initial conditions (position, velocity, attitude).
- `vehicle` — vehicle model to simulate (default `QuadrotorMini3`).
- `target` — static flight goal.
- `trajectory` — optional time-varying trajectory; when present, overrides the static `target`.
- `disturbances` — list of disturbance configurations (wind gust, turbulence, motor failure).
- `assertions` — list of assertions — scenario pass/fail criteria.

#### `Scenario::from_file`

```rust path=null start=null
pub fn from_file(path: &std::path::Path) -> Result<Self, ScenarioError>
```

Loads a scenario from a TOML file. Returns `ScenarioError::Io` on I/O errors or `ScenarioError::Toml` on parse errors.

#### `FromStr` impl

```rust path=null start=null
impl std::str::FromStr for Scenario {
    type Err = ScenarioError;
    fn from_str(s: &str) -> Result<Self, ScenarioError>;
}
```

Parses a scenario directly from a TOML string. Allows `s.parse::<Scenario>()`.

### `ScenarioError`

```rust path=null start=null
pub enum ScenarioError {
    Io(std::io::Error),
    Toml(toml::de::Error),
}
```

- `Io` — file read error (file not found, permission denied).
- `Toml` — TOML syntax error.

### `VehicleKind`

Enum selecting the vehicle model to simulate.

```rust path=null start=null
pub enum VehicleKind {
    QuadrotorMini3,
    QuadrotorMini3Simple,
    F16,
}
```

Variants:

- `QuadrotorMini3` *(default)* — DJI Mini 3 quadrotor with the full model (ISA atmosphere, motor dynamics).
- `QuadrotorMini3Simple` — simplified DJI Mini 3 (constant atmosphere density, faster linearisation).
- `F16` — F-16A jet aircraft (NASA TP-1538 aerodynamic model, F110 engine).

In TOML: `vehicle = "quadrotor_mini3"`, `"quadrotor_mini3_simple"`, `"f16"`.

### `ScenarioTarget`

The flight goal in a scenario.

```rust path=null start=null
pub struct ScenarioTarget {
    pub z: f64,
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub yaw: Option<f64>,
}
```

Fields:

- `z` — target altitude [m] — **required**.
- `x` — target X position [m] — optional (default: no X control).
- `y` — target Y position [m] — optional.
- `yaw` — target yaw angle [rad] — optional.

### `InitialConditions`

Simulation initial conditions.

```rust path=null start=null
pub struct InitialConditions {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude_deg: [f64; 3],
}
```

Fields:

- `position` — initial position `[x, y, z]` [m] (default `[0, 0, 0]`).
- `velocity` — initial velocity `[vx, vy, vz]` [m/s] (default `[0, 0, 0]`).
- `attitude_deg` — initial attitude `[roll, pitch, yaw]` [°] (default `[0, 0, 0]`). Required e.g. for the F-16, which starts at angle of attack α = 5°.

### `Assertion`

A single scenario assertion (pass/fail criterion).

```rust path=null start=null
pub struct Assertion {
    pub metric: MetricKind,
    pub max: f64,
}
```

Fields:

- `metric` — metric kind to check.
- `max` — maximum allowed metric value. The assertion passes when `value ≤ max`.

### `MetricKind`

Enum specifying the type of control quality metric. Used in scenario assertions.

```rust path=null start=null
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
```

Variants:

- `PositionRms3d` — RMS 3D position error.
- `PositionRmsAxis(Axis)` — RMS position error along one axis (X, Y, or Z).
- `PositionMaxError3d` — maximum 3D position error.
- `PositionMaxErrorAxis(Axis)` — maximum position error along one axis.
- `VelocityRms3d` — RMS 3D velocity (deviation from rest).
- `VelocityRmsAxis(Axis)` — RMS velocity along one axis.
- `AttitudeRms` — RMS attitude error (roll² + pitch²) [rad].
- `AttitudeMaxError` — maximum attitude error √(roll² + pitch²) [rad].
- `OvershootPercent` — overshoot expressed as a percentage of the step range [%].
- `SettlingTimeS` — settling time (0.1 m band) [s].
- `RiseTimeS` — rise time (10%–90% of step range) [s].
- `SteadyStateError` — steady-state error (mean error over the last 20% of simulation) [m].
- `ControlEnergy` — control energy consumption (ω³ approximation) [a.u.].
- `MaxControlRate` — maximum rate of change of the control signal [rad/s²].

Helper enum `Axis`:

```rust path=null start=null
pub enum Axis { X, Y, Z }
```

In TOML, axis metrics are written as e.g.: `metric = { position_rms_axis = "z" }`.

### `ScenarioTrajectoryDef`

Time-varying trajectory definition. The `type` field in TOML selects the variant.

```rust path=null start=null
pub enum ScenarioTrajectoryDef {
    Hold { z: f64, x: Option<f64>, y: Option<f64>, yaw: Option<f64> },
    Waypoint { waypoints: Vec<WaypointEntry> },
    Circle { cx: f64, cy: f64, radius: f64, omega_deg_s: f64, altitude_m: f64 },
}
```

Variants:

- `Hold` — hold a fixed position. Fields: `z` (required), `x`, `y`, `yaw` (optional).
- `Waypoint` — linear path through timed waypoints. Field `waypoints` — list of `WaypointEntry`.
- `Circle` — circular horizontal orbit. Fields:
  - `cx`, `cy` — orbit centre [m].
  - `radius` — radius [m].
  - `omega_deg_s` — angular velocity [°/s] (converted to rad/s internally).
  - `altitude_m` — orbit altitude [m].

The `into_trajectory()` method converts the definition into a `Box<dyn Trajectory>` object.

### `WaypointEntry`

A single waypoint in a `Waypoint` trajectory.

```rust path=null start=null
pub struct WaypointEntry {
    pub time_s: f64,
    pub z: f64,
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub yaw: Option<f64>,
}
```

Fields:

- `time_s` — time at which this waypoint should be reached [s].
- `z` — altitude [m] — **required**.
- `x` — X position [m] — optional.
- `y` — Y position [m] — optional.
- `yaw` — yaw angle [rad] — optional.

### Complete TOML Example

```toml path=null start=null
name = "hover_z5_with_gust"
description = "Climb to 5m with a wind gust at 3 seconds"
duration_s = 10.0
dt_s = 0.005
vehicle = "quadrotor_mini3"

[initial]
position = [0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0]
attitude_deg = [0.0, 0.0, 0.0]

[target]
z = 5.0
x = 1.0
yaw = 0.5

[[disturbances]]
type = "wind_gust"
at_s = 3.0
duration_s = 0.2
force = [2.0, 0.0, -1.0]

[[disturbances]]
type = "turbulence"
start_s = 4.0
end_s = 8.0
intensity_n = 0.3
seed = 42
z_only = false

[[assertions]]
metric = "overshoot_percent"
max = 15.0

[[assertions]]
metric = "settling_time_s"
max = 5.0

[[assertions]]
metric = "position_rms3d"
max = 1.5

[[assertions]]
metric = "steady_state_error"
max = 0.05
```

Circular trajectory example:

```toml path=null start=null
name = "circle_orbit"
duration_s = 30.0
dt_s = 0.01

[initial]
position = [5.0, 0.0, 3.0]

[target]
z = 3.0

[trajectory]
type = "circle"
cx = 0.0
cy = 0.0
radius = 5.0
omega_deg_s = 30.0
altitude_m = 3.0

[[assertions]]
metric = "position_rms3d"
max = 0.5
```

---

## 3. Module `controller_config`

Contains controller configuration types and factories that create controller instances.

### `ControllerConfig`

Main enum selecting and configuring the controller. Deserialised from TOML with an internal `type` tag.

```rust path=null start=null
pub enum ControllerConfig {
    Cascade(CascadeConfig),
    Lqr(LqrConfig),
    Lqi(LqiConfig),
}
```

Variants:

- `Cascade` — cascade PID controller. TOML: `type = "cascade"`.
- `Lqr` — Linear-Quadratic Regulator. TOML: `type = "lqr"`.
- `Lqi` — LQR with integral action. TOML: `type = "lqi"`.

Default implementation (`Default`) returns `Cascade(CascadeConfig::default())`.

#### `name()`

```rust path=null start=null
pub fn name(&self) -> &str
```

Returns a human-readable controller name: `"Cascade-PID"`, `"LQR"`, or `"LQI"`.

#### `from_file()`

```rust path=null start=null
pub fn from_file(path: &Path) -> anyhow::Result<Self>
```

Loads the controller configuration from a TOML file.

#### `into_factory()`

```rust path=null start=null
pub fn into_factory(self) -> ControllerFactory
```

Converts the configuration into a closure (`ControllerFactory`) that creates a fresh controller instance. The factory takes a reference to the vehicle model (needed to determine the equilibrium point / linearisation) and returns `Box<dyn Controller>`.

### `PidConfig`

Parameters for a single PID loop.

```rust path=null start=null
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limit: f64,
    pub output_limit: f64,
}
```

Fields:

- `kp` — proportional gain.
- `ki` — integral gain.
- `kd` — derivative gain.
- `integral_limit` — anti-windup clamp on the integral accumulator.
- `output_limit` — clamp on the loop output.

### `CascadeConfig`

Cascade PID controller configuration. The cascade has three levels: position → velocity (outer), velocity → attitude (middle), attitude → motor commands (inner).

```rust path=null start=null
pub struct CascadeConfig {
    pub max_tilt_deg: f64,
    pub vel_z: PidConfig,
    pub vel_xy: PidConfig,
    pub att: PidConfig,
    pub att_yaw: PidConfig,
}
```

Fields:

- `max_tilt_deg` — maximum horizontal tilt [°]. Default 8.6° — prevents motor saturation during combined roll and pitch.
- `vel_z` — vertical velocity loop: vz error → delta throttle.
- `vel_xy` — horizontal velocity loop: vx/vy error → target pitch/roll. Shared configuration for both X and Y axes.
- `att` — attitude loop: roll/pitch error → delta motor command. Shared configuration for roll and pitch.
- `att_yaw` — yaw attitude loop: yaw error → delta motor command.

Default values (`Default`):

- `max_tilt_deg = 8.6`
- `vel_z`: kp=0.3, ki=0.1, kd=0.0, integral_limit=0.45, output_limit=0.45
- `vel_xy`: kp=0.4, ki=0.05, kd=0.0, integral_limit=0.5, output_limit=0.35
- `att`: kp=4.0, ki=0.0, kd=0.2, integral_limit=1.0, output_limit=1.0
- `att_yaw`: kp=2.0, ki=0.1, kd=0.0, integral_limit=0.5, output_limit=0.5

### `LqrConfig`

LQR (Linear-Quadratic Regulator) configuration. The CARE equation is solved once around a fixed equilibrium. LQR stabilises the trim point — it does **not** track arbitrary setpoints (use `LqiConfig` for tracking).

Supported only for quadrotor vehicles.

```rust path=null start=null
pub struct LqrConfig {
    pub trim_z_m: f64,
    pub q_weights: Option<Vec<f64>>,
    pub r_weights: Option<Vec<f64>>,
}
```

Fields:

- `trim_z_m` — trim altitude for linearisation [m]. Default 5.0.
- `q_weights` — Q weight vector — 13 elements for a quadrotor: `[x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]`. Optional — built-in defaults: `[3.0, 3.0, 80.0, 0.5, 0.5, 10.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0]`.
- `r_weights` — R weight vector — 4 elements (one per motor). Larger values → smoother, less aggressive control. Default `[0.01, 0.01, 0.01, 0.01]`.

### `LqiConfig`

LQI (Linear-Quadratic Integral) controller configuration. Extends LQR with four integral states `[ξ_x, ξ_y, ξ_z, ξ_ψ]` that eliminate steady-state error under constant disturbances.

Supported only for quadrotor vehicles.

```rust path=null start=null
pub struct LqiConfig {
    pub trim_z_m: f64,
    pub q_weights: Option<Vec<f64>>,
    pub r_weights: Option<Vec<f64>>,
    pub xi_limits: Option<[f64; 4]>,
}
```

Fields:

- `trim_z_m` — trim altitude for linearisation [m]. Default 5.0.
- `q_weights` — Q weight vector — 17 elements: 13 plant states + 4 integral states `[ξ_x, ξ_y, ξ_z, ξ_ψ]`. Defaults: `[1.0, 1.0, 100.0, 0.5, 0.5, 12.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0, 5.0, 5.0, 6.0, 2.0]`.
- `r_weights` — R weight vector — 4 elements. Default `[0.005, 0.005, 0.005, 0.005]`.
- `xi_limits` — anti-windup limits `[m·s, m·s, m·s, rad·s]` on the four integral states. Default `[30, 30, 30, 2π]`.

---

## 4. Module `runner`

SITL simulation loop. Combines a scenario, vehicle model, controller, and disturbances into a single simulation.

### `ControllerFactory`

Type alias — a factory that creates a fresh controller for a given vehicle model.

```rust path=null start=null
pub type ControllerFactory =
    Box<dyn Fn(&dyn VehicleModel) -> Result<Box<dyn Controller>> + Send + Sync>;
```

The factory (rather than a ready-made instance) guarantees that each simulation run starts with a clean controller state.

### `run_scenario()`

```rust path=null start=null
pub fn run_scenario(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
) -> Result<ScenarioReport>
```

Runs a SITL scenario. If the scenario defines a trajectory, it is used automatically (overrides the static `[target]`). Otherwise the static goal is used.

Parameters:

- `scenario` — scenario definition (duration, goal, disturbances, assertions).
- `model` — vehicle model (e.g. `QuadrotorModel::mini3()`).
- `factory` — controller factory.

Returns a `ScenarioReport` with assertion results and frame history.

### `run_scenario_with_trajectory()`

```rust path=null start=null
pub fn run_scenario_with_trajectory(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
    trajectory: &dyn Trajectory,
) -> Result<ScenarioReport>
```

Runs a scenario with a time-varying trajectory instead of a static goal. At each simulation step `trajectory.target(time_s)` is called. Assertions are evaluated against the trajectory's final goal.

### Internal functions (`pub(crate)`)

#### `run_with_disturbances()`

Canonical simulation loop. At each step:
1. Applies active disturbances (`disturbance.apply()`).
2. Calls the controller (`controller.update()`).
3. Updates actuator dynamics (`model.step_actuators()`).
4. Integrates state with RK4.

Returns the full frame history `Vec<SimFrame>`.

#### `run_with_disturbances_traj()`

Like `run_with_disturbances`, but calls `trajectory.target(time)` at each step instead of a fixed goal.

#### `scenario_to_flight_target()`

Converts `ScenarioTarget` into a `FlightTarget` from the control library. The `z` field (required in TOML) is always `Some`; `x`, `y`, `yaw` remain `None` if absent — meaning no control on that axis.

---

## 5. Module `disturbance`

Models external disturbances acting on the drone during simulation.

### Trait `Disturbance`

```rust path=null start=null
pub trait Disturbance: Send + Sync {
    fn is_active(&self, time: f64) -> bool;
    fn apply(&self, state: &mut DroneState, model: &dyn VehicleModel, dt: TimeStep);
}
```

Methods:

- `is_active(time)` — checks whether the disturbance is active at time `time` [s].
- `apply(state, model, dt)` — modifies the drone state (`DroneState`) according to the disturbance model. Receives the vehicle model (e.g. to fetch mass) and the time step.

### `DisturbanceConfig`

Configuration enum deserialised from TOML. Internal `type` tag selects the variant.

```rust path=null start=null
pub enum DisturbanceConfig {
    WindGust(WindGustConfig),
    Turbulence(TurbulenceConfig),
    MotorFailure(MotorFailureConfig),
}
```

The `into_disturbance()` method creates the appropriate `Box<dyn Disturbance>` instance.

### `WindGust` / `WindGustConfig`

An impulsive wind force acting over a specified time window.

```rust path=null start=null
pub struct WindGustConfig {
    pub at_s: f64,
    pub duration_s: f64,  // default 0.1
    pub force: [f64; 3],
}
```

Fields:

- `at_s` — gust start time [s].
- `duration_s` — gust duration [s] (default 0.1 s).
- `force` — force vector `[Fx, Fy, Fz]` [N].

Active in the interval `[at_s, at_s + duration_s)`. Effect: adds velocity change `Δv = F · dt / m` to the drone state (force impulse).

TOML:
```toml path=null start=null
[[disturbances]]
type = "wind_gust"
at_s = 3.0
duration_s = 0.2
force = [2.0, 0.0, -1.0]
```

### `Turbulence` / `TurbulenceConfig`

Continuous Gaussian noise modelling atmospheric turbulence.

```rust path=null start=null
pub struct TurbulenceConfig {
    pub start_s: f64,
    pub end_s: f64,
    pub intensity_n: f64,
    pub seed: u64,
    pub z_only: bool,
}
```

Fields:

- `start_s` — turbulence start time [s].
- `end_s` — turbulence end time [s].
- `intensity_n` — intensity (standard deviation of the normal force distribution) [N].
- `seed` — PRNG seed (default 0). Ensures deterministic reproducibility.
- `z_only` — if `true`, turbulence acts only on the Z axis; X/Y remain undisturbed. Useful for testing altitude disturbance rejection without XY→Z coupling.

Active in `[start_s, end_s)`. Each simulation step draws a force from N(0, intensity_n²) and adds `Δv = F · dt / m`. Generator: `SmallRng` (fast, deterministic).

### `MotorFailure` / `MotorFailureConfig`

Permanent failure of one motor. Simulates an unbalanced yaw torque.

```rust path=null start=null
pub struct MotorFailureConfig {
    pub at_s: f64,
    pub motor_index: usize,
}
```

Fields:

- `at_s` — failure time [s].
- `motor_index` — failed motor index (0–3 for a quadrotor).

Active from `at_s` to the end of the simulation (permanent). Adds a yaw torque proportional to the square of the motor speed at equilibrium (`k_torque ≈ 1.5e-8`). The sign of the torque depends on the motor index parity.

---

## 6. Module `metrics`

Functions that compute control quality metrics from the simulation frame history.

### `compute()`

```rust path=null start=null
pub fn compute(metric: &MetricKind, frames: &[SimFrame], target: &FlightTarget) -> f64
```

Dispatcher — calls the appropriate metric function based on the `MetricKind` variant. Returns the metric value as `f64`.

### Position Metrics

#### `position_rms_3d()`

```rust path=null start=null
pub fn position_rms_3d(frames: &[SimFrame], target: &FlightTarget) -> f64
```

RMS 3D position error.

Formula: `√( (1/N) · Σ ||p_i - p_target||² )`

#### `position_rms_axis()`

```rust path=null start=null
pub fn position_rms_axis(frames: &[SimFrame], target: &FlightTarget, axis: &Axis) -> f64
```

RMS position error along one axis (X, Y, or Z).

Formula: `√( (1/N) · Σ (p_i[axis] - p_target[axis])² )`

#### `position_rms_z()`

```rust path=null start=null
pub fn position_rms_z(frames: &[SimFrame], target: &FlightTarget) -> f64
```

RMS Z position error (altitude). Legacy version used in `comparison.rs`.

#### `position_max_error_3d()`

```rust path=null start=null
pub fn position_max_error_3d(frames: &[SimFrame], target: &FlightTarget) -> f64
```

Maximum 3D position error over the entire flight.

Formula: `max( ||p_i - p_target|| )`

#### `position_max_error_axis()`

```rust path=null start=null
pub fn position_max_error_axis(frames: &[SimFrame], target: &FlightTarget, axis: &Axis) -> f64
```

Maximum position error along one axis.

Formula: `max( |p_i[axis] - p_target[axis]| )`

### Velocity Metrics

Reference = 0 (deviation from rest).

#### `velocity_rms_3d()`

```rust path=null start=null
pub fn velocity_rms_3d(frames: &[SimFrame]) -> f64
```

RMS 3D velocity.

Formula: `√( (1/N) · Σ ||v_i||² )`

#### `velocity_rms_axis()`

```rust path=null start=null
pub fn velocity_rms_axis(frames: &[SimFrame], axis: &Axis) -> f64
```

RMS velocity along one axis.

### Attitude Metrics

Reference = level flight (roll = 0, pitch = 0).

#### `attitude_rms()`

```rust path=null start=null
pub fn attitude_rms(frames: &[SimFrame]) -> f64
```

RMS attitude error.

Formula: `√( (1/N) · Σ (roll² + pitch²) )` [rad]

#### `attitude_max_error()`

```rust path=null start=null
pub fn attitude_max_error(frames: &[SimFrame]) -> f64
```

Maximum attitude error.

Formula: `max( √(roll² + pitch²) )` [rad]

### Step Response Metrics

#### `overshoot_percent()`

```rust path=null start=null
pub fn overshoot_percent(frames: &[SimFrame], target: &FlightTarget) -> f64
```

Overshoot — by what percentage the drone exceeded the target relative to the total step range.

Formula: `((z_max - z_target) / (z_target - z_initial)) · 100%`

Returns 0.0 if the drone never reached or exceeded the target.

#### `settling_time_s()`

```rust path=null start=null
pub fn settling_time_s(frames: &[SimFrame], target: &FlightTarget) -> f64
```

Settling time — time [s] of the last moment when Z error ≥ 0.1 m (fixed threshold = 0.1 m). After this time the drone remains within the tolerance band.

#### `rise_time_s()`

```rust path=null start=null
pub fn rise_time_s(frames: &[SimFrame], target_z: f64) -> f64
```

Rise time — time to travel from 10% to 90% of the total Z position change.

Formula: `t_90% - t_10%`

Returns `f64::INFINITY` if 90% was not reached.

#### `steady_state_error()`

```rust path=null start=null
pub fn steady_state_error(frames: &[SimFrame], target_z: f64) -> f64
```

Steady-state error — mean absolute Z error over the last 20% of simulation time.

Formula: `(1/N_late) · Σ |z_i - z_target|` for frames with `t ≥ 0.8 · t_max`

### Energy Metrics

#### `control_energy()`

```rust path=null start=null
pub fn control_energy(frames: &[SimFrame]) -> f64
```

Approximation of the total energy consumed by the motors.

In the propeller model, torque τ = k_torque · ω², so mechanical power P = τ · ω = k_torque · **ω³**. The integral Σ ω³ · dt over all motors gives a quantity proportional to energy (k_torque cancels out when comparing controllers on the same vehicle).

Using ω³ (rather than the naive ω²) ensures correct ranking for profiles combining short high-RPM bursts with sustained moderate thrust.

#### `max_control_rate()`

```rust path=null start=null
pub fn max_control_rate(frames: &[SimFrame]) -> f64
```

Maximum control signal rate of change — max of `|ω[i](t+1) - ω[i](t)| / dt` over all motors and time steps [rad/s²].

---

## 7. Module `report`

Structures reporting the results of a single scenario.

### `AssertionResult`

Result of a single assertion.

```rust path=null start=null
pub struct AssertionResult {
    pub metric: String,
    pub value: f64,
    pub max: f64,
    pub passed: bool,
}
```

Fields:

- `metric` — metric name (e.g. `"OvershootPercent"`).
- `value` — computed metric value.
- `max` — threshold from the scenario assertion.
- `passed` — `true` if `value ≤ max`.

### `ScenarioReport`

Full scenario report.

```rust path=null start=null
pub struct ScenarioReport {
    pub name: String,
    pub passed: bool,
    pub duration_s: f64,
    pub frame_count: usize,
    pub assertions: Vec<AssertionResult>,
    pub frames: Vec<SimFrame>,
}
```

Fields:

- `name` — scenario name.
- `passed` — `true` if all assertions passed.
- `duration_s` — simulation duration [s].
- `frame_count` — number of frames in the history.
- `assertions` — individual assertion results.
- `frames` — full simulation frame history.

#### `print()`

```rust path=null start=null
pub fn print(&self)
```

Prints the report to stdout in a human-readable format (PASS/FAIL status, timings, assertion results with ✓/✗ symbols).

#### `to_csv()`

```rust path=null start=null
pub fn to_csv(&self) -> String
```

Serialises the frame history to CSV format. Columns: `time,x,y,z,vx,vy,vz`.

---

## 8. Module `comparison`

Comparing multiple controllers on the same scenario.

### `ControllerResult`

Result of one controller in a comparison.

```rust path=null start=null
pub struct ControllerResult {
    pub name: String,
    pub frames: Vec<SimFrame>,
    pub rms_error_z: f64,
    pub max_error_z: f64,
    pub overshoot_pct: f64,
    pub settling_time_s: f64,
    pub rise_time_s: f64,
    pub steady_state_err: f64,
    pub control_energy: f64,
    pub max_control_rate: f64,
}
```

Fields:

- `name` — controller name.
- `frames` — simulation frame history.
- `rms_error_z` — RMS Z error [m].
- `max_error_z` — maximum Z error [m].
- `overshoot_pct` — overshoot [%].
- `settling_time_s` — settling time [s].
- `rise_time_s` — rise time [s].
- `steady_state_err` — steady-state error [m].
- `control_energy` — control energy [a.u.].
- `max_control_rate` — max control rate [rad/s²].

### `ComparisonReport`

Aggregate comparison report.

```rust path=null start=null
pub struct ComparisonReport {
    pub scenario_name: String,
    pub target_z: f64,
    pub results: Vec<ControllerResult>,
}
```

Fields:

- `scenario_name` — scenario name.
- `target_z` — target altitude [m].
- `results` — per-controller results.

#### `print_table()`

Prints a comparison table to stdout with columns: Controller, RMS Z [m], OS [%], ST [s], RT [s], Energy.

#### `to_csv_trajectories()`

Exports all controller trajectories to CSV. Columns: `time`, `{name}_z` per controller.

#### `to_csv_metrics()`

Exports the metrics table to CSV. One row per controller.

### `run_comparison()`

```rust path=null start=null
pub fn run_comparison(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    controllers: Vec<(String, ControllerFactory)>,
) -> Result<ComparisonReport>
```

Runs all controllers on the same scenario and collects their results into a `ComparisonReport`.

---

## 9. Module `monte_carlo`

Monte Carlo analysis with perturbed initial conditions.

### `MonteCarloConfig`

Monte Carlo run configuration.

```rust path=null start=null
pub struct MonteCarloConfig {
    pub runs: usize,
    pub pos_noise_m: f64,
    pub vel_noise_ms: f64,
    pub seed: u64,
}
```

Fields:

- `runs` — number of independent simulation runs.
- `pos_noise_m` — standard deviation of initial position noise [m].
- `vel_noise_ms` — standard deviation of initial velocity noise [m/s].
- `seed` — PRNG seed.

Default values: `runs = 100`, `pos_noise_m = 0.5`, `vel_noise_ms = 0.1`, `seed = 42`.

### `MonteCarloReport`

Monte Carlo results report.

```rust path=null start=null
pub struct MonteCarloReport {
    pub scenario_name: String,
    pub runs: usize,
    pub metrics: Vec<MetricStats>,
}
```

#### `MetricStats`

Per-metric statistics.

```rust path=null start=null
pub struct MetricStats {
    pub name: String,
    pub mean: f64,
    pub std: f64,
    pub min: f64,
    pub max: f64,
}
```

Fields:

- `name` — metric name.
- `mean` — mean value.
- `std` — standard deviation.
- `min` / `max` — minimum and maximum observed value.

#### `MonteCarloReport::print()`

Prints a statistics table to stdout.

#### `MonteCarloReport::to_csv()`

Exports per-run metric values to CSV.

### `run_monte_carlo()`

```rust path=null start=null
pub fn run_monte_carlo(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
    config: MonteCarloConfig,
) -> Result<MonteCarloReport>
```

Runs N independent simulations with perturbed initial conditions and aggregates metric statistics.

**Behaviour:**
1. For each run, draws Gaussian noise and adds it to the initial position and velocity.
2. Runs all simulations in parallel (Rayon).
3. Aggregates statistics across runs.
