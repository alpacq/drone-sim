# drone-control — Reference Documentation

## 1. Overview

The `drone-control` crate provides a complete flight controller stack for the drone simulator. It includes:

- **`Controller` trait** — unified flight controller interface.
- **`FlightTarget`** — description of the commanded flight state with optional axes.
- **PID controller** with anti-windup protection.
- **Inner loop** — velocity error to control signal conversion.
- **Velocity profiler** — outer loop: position → commanded velocity.
- **Mixer** — translates attitude commands into actuator signals.
- **Cascade controller** — three-level cascade: position → velocity → angles → motors.
- **LQR / LQI** — linear-quadratic regulator with optional integral action to eliminate steady-state error.
- **Trajectories** — time-varying flight target generators.

---

## 2. Module `controller`

Defines the common trait for all flight controllers.

### Trait `Controller`

```rust
pub trait Controller: Send + Sync {
    fn update(&mut self, state: &DroneState, target: &FlightTarget, dt: TimeStep) -> KnownActuatorInput;
    fn reset(&mut self);
    fn name(&self) -> &str;
}
```

#### Methods

- **`update(&mut self, state: &DroneState, target: &FlightTarget, dt: TimeStep) -> KnownActuatorInput`**
  Computes the controller output (actuator signals) from the current drone state `state`, flight target `target`, and time step `dt`. Called every simulation step.

- **`reset(&mut self)`**
  Resets the controller's internal state (integrals, previous errors). Used when changing flight modes or restarting the controller.

- **`name(&self) -> &str`**
  Returns the controller name (e.g. `"CascadeController"`, `"LQR"`, `"LqiController"`). Used for logging and diagnostics.

---

## 3. Module `target`

Describes the commanded flight state as a set of optional setpoints per axis.

### Struct `FlightTarget`

```rust
#[derive(Debug, Clone)]
pub struct FlightTarget {
    pub x:   Option<f64>,
    pub y:   Option<f64>,
    pub z:   Option<f64>,
    pub yaw: Option<f64>,
}
```

#### Fields

- **`x: Option<f64>`** — commanded X position [m]. `None` = X axis not controlled.
- **`y: Option<f64>`** — commanded Y position [m]. `None` = Y axis not controlled.
- **`z: Option<f64>`** — commanded altitude Z [m]. `None` = altitude not controlled.
- **`yaw: Option<f64>`** — commanded yaw angle [rad]. `None` = yaw not controlled.

Semantics of `None`: the cascade controller and LQI integrators treat a missing axis as zero error and zero integral accumulation — the drone stabilises at its current position on that axis rather than driving to zero.

#### Factory methods

- **`FlightTarget::altitude(z: f64) -> Self`**
  Altitude-only target. Only Z is controlled; X, Y, and yaw set to `None`.

- **`FlightTarget::position(x: f64, y: f64, z: f64) -> Self`**
  3D position target (no yaw control). X, Y, Z set to `Some`; yaw = `None`.

- **`FlightTarget::full(x: f64, y: f64, z: f64, yaw: f64) -> Self`**
  Full 3D + yaw target. All four axes set to `Some`.

---

## 4. Module `pid`

PID controller implementation with anti-windup protection.

### Struct `Pid`

```rust
#[derive(Debug, Clone)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limit: f64,
    pub output_limit: f64,
    // private fields: integral, prev_error
}
```

#### Public fields

- **`kp: f64`** — proportional gain.
- **`ki: f64`** — integral gain.
- **`kd: f64`** — derivative gain.
- **`integral_limit: f64`** — maximum absolute value of the integral term (anti-windup).
- **`output_limit: f64`** — maximum absolute value of the controller output.

#### Methods

- **`Pid::new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self`**
  Creates a new PID controller with the given parameters. Internal state initialised to zero.

- **`update(&mut self, error: f64, dt: TimeStep) -> f64`**
  Computes the controller output for the given error and time step:
  - P = `kp × error`
  - I = `ki × ∫error·dt` (clamped to `±integral_limit`)
  - D = `kd × (error − prev_error) / dt`
  - Output = `clamp(P + I + D, ±output_limit)`

- **`reset(&mut self)`**
  Resets internal state (`integral = 0`, `prev_error = 0`).

---

## 5. Module `inner_loop`

Inner loop of the cascade controller: velocity error → control signal. Has internal state (memory).

### Trait `InnerLoop`

```rust
pub trait InnerLoop: Send + Sync {
    fn compute(&mut self, error: f64, dt: TimeStep) -> f64;
    fn reset(&mut self);
}
```

#### Methods

- **`compute(&mut self, error: f64, dt: TimeStep) -> f64`**
  Computes the control output for the given error and time step.

- **`reset(&mut self)`**
  Resets the loop's internal state.

### Struct `PidLoop`

```rust
pub struct PidLoop(pub Pid);
```

Wraps `Pid` and implements the `InnerLoop` trait. Delegates `compute` to `Pid::update` and `reset` to `Pid::reset`.

#### Methods

- **`PidLoop::new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self`**
  Creates a new `PidLoop` with the given PID parameters.

---

## 6. Module `profiler`

Outer loop of the cascade controller: position → commanded velocity. Profilers are stateless — the same input always produces the same output.

### Trait `VelocityProfiler`

```rust
pub trait VelocityProfiler: Send + Sync {
    fn compute(&self, error: f64) -> f64;
}
```

#### Methods

- **`compute(&self, error: f64) -> f64`**
  Computes the commanded velocity [m/s] for the given position error [m].

### Struct `SqrtProfiler`

Square-root profiler — kinematic braking profile.

```rust
pub struct SqrtProfiler {
    pub brake_accel: f64,
    pub v_max: f64,
}
```

#### Fields

- **`brake_accel: f64`** — braking deceleration [m/s²].
- **`v_max: f64`** — maximum approach speed [m/s].

#### Formula

```
v = sign(e) × min(√(2 · brake_accel · |e|), v_max)
```

For small errors the speed grows gently (proportional to √|e|); for large errors it is capped at `v_max`. Provides smooth braking without overshooting the target.

#### Methods

- **`SqrtProfiler::new(brake_accel: f64, v_max: f64) -> Self`**
  Creates a profiler with the given parameters.

- **`SqrtProfiler::for_altitude() -> Self`**
  Preset profiler for the Z axis: `brake_accel = 1.5 m/s²`, `v_max = 1.0 m/s`.

- **`SqrtProfiler::for_horizontal() -> Self`**
  Preset profiler for the XY plane: `brake_accel = 2.0 m/s²`, `v_max = 3.0 m/s`.

### Struct `LinearProfiler`

Linear profiler — simple proportional relationship.

```rust
pub struct LinearProfiler {
    pub kp: f64,
    pub v_max: f64,
}
```

#### Fields

- **`kp: f64`** — proportional gain.
- **`v_max: f64`** — maximum speed [m/s].

#### Formula

```
v = clamp(kp × error, -v_max, v_max)
```

#### Methods

- **`LinearProfiler::new(kp: f64, v_max: f64) -> Self`**
  Creates a linear profiler with the given parameters.

---

## 7. Module `mixer`

Translates high-level attitude commands into concrete actuator signals.

### Struct `AttitudeCommand`

```rust
#[derive(Debug, Clone, Copy)]
pub struct AttitudeCommand {
    pub throttle: f64,
    pub roll:     f64,
    pub pitch:    f64,
    pub yaw:      f64,
}
```

#### Fields

- **`throttle: f64`** — throttle [0, 1].
- **`roll: f64`** — roll command [-1, 1].
- **`pitch: f64`** — pitch command [-1, 1].
- **`yaw: f64`** — yaw command [-1, 1].

### Trait `Mixer`

```rust
pub trait Mixer: Send + Sync {
    fn mix(&self, cmd: &AttitudeCommand) -> KnownActuatorInput;
    fn equilibrium_command(&self) -> AttitudeCommand;
}
```

#### Methods

- **`mix(&self, cmd: &AttitudeCommand) -> KnownActuatorInput`**
  Translates an attitude command into actuator signals (motor speeds for a quadrotor or control surface deflections for an aircraft).

- **`equilibrium_command(&self) -> AttitudeCommand`**
  Returns the command corresponding to the equilibrium state (hover/cruise). Used as the operating point in the cascade controller.

### Struct `QuadrotorMixer`

Mixer for a quadrotor in X-frame configuration.

```rust
pub struct QuadrotorMixer {
    hover_motor_speed: f64,
    max_motor_speed: f64,
}
```

#### X-frame geometry (top-down view)

```
  1(CCW)  0(CW)
     \   /
      [B]     ← front (+x)
     /   \
  2(CW)  3(CCW)
```

- Motor 0 (FrontRight, CW): `base - p - r + y`
- Motor 1 (FrontLeft, CCW): `base - p + r - y`
- Motor 2 (RearLeft, CW): `base + p + r + y`
- Motor 3 (RearRight, CCW): `base + p - r - y`

Where `base = throttle × max_motor_speed`, `r/p/y = roll/pitch/yaw × max_motor_speed × 0.5`.

#### Methods

- **`QuadrotorMixer::new(hover_motor_speed: f64, max_motor_speed: f64) -> Self`**
  Creates a mixer with the given motor speeds.

- **`QuadrotorMixer::from_equilibrium(input: KnownActuatorInput) -> Self`**
  Creates a mixer from an equilibrium input (mean hover motor speed). Panics if the input is not the `Quadrotor` variant.

### Struct `FixedWingMixer`

Mixer for fixed-wing aircraft.

```rust
pub struct FixedWingMixer {
    cruise_throttle: f64,
}
```

Translates `AttitudeCommand` directly into `KnownActuatorInput::FixedWing` with clamping to valid ranges: throttle [0, 1], aileron/elevator/rudder [-1, 1].

#### Methods

- **`FixedWingMixer::new(cruise_throttle: f64) -> Self`**
  Creates a mixer with the given cruise throttle.

- **`FixedWingMixer::from_equilibrium(input: KnownActuatorInput) -> Self`**
  Creates a mixer from an equilibrium input. Panics if the input is not the `FixedWing` variant.

---

## 8. Module `cascade`

Three-level cascade flight controller with full XYZ + yaw control.

### Struct `CascadeController<Pz, Pxy, I>`

```rust
pub struct CascadeController<Pz, Pxy, I>
where
    Pz:  VelocityProfiler,
    Pxy: VelocityProfiler,
    I:   InnerLoop,
```

#### Generic parameters

- **`Pz`** — velocity profiler for the Z axis (altitude). Implements `VelocityProfiler`.
- **`Pxy`** — velocity profiler for the XY axes (horizontal). May differ from `Pz` — e.g. `SqrtProfiler` for Z and `LinearProfiler` for XY, without heap allocation.
- **`I`** — inner loop implementation. Implements `InnerLoop`.

#### Public fields

- **`max_tilt_rad: f64`** — maximum tilt angle for XY control [rad]. Default `0.15` (~8.6°). Prevents motor saturation when roll and pitch are commanded simultaneously.
- **`tilt_compensation: bool`** — compensation for thrust loss due to body tilt. Default `true`. Divides throttle by `cos(roll) × cos(pitch)` (with a lower bound of 0.3).

#### Private fields (set by the constructor)

- `profiler_z` / `profiler_xy` — velocity profilers (outer loop).
- `vel_loop_z`, `vel_loop_x`, `vel_loop_y` — velocity loops (middle loop): vZ → delta throttle, vX → target pitch, vY → target roll.
- `att_loop_roll`, `att_loop_pitch`, `att_loop_yaw` — attitude loops (inner loop).
- `mixer: Box<dyn Mixer>` — actuator mixer.

#### Control cascade

The `update()` algorithm implements three cascade levels:

1. **Position → velocity** (outer loop): Position error on each active axis passes through the corresponding profiler (`profiler_z` for Z, `profiler_xy` for XY), producing a commanded velocity. Axes with `None` in `FlightTarget` produce zero velocity command.

2. **Velocity → angles / throttle** (middle loop): Velocity error passes through PID loops:
   - vZ → delta throttle (added to equilibrium throttle)
   - vX → target pitch (clamped to `±max_tilt_rad`)
   - vY → target roll (negated — positive roll produces thrust in −Y)

3. **Angles → motors** (inner loop): Attitude error (target − current Euler) passes through PID loops, and the result goes to the mixer as an `AttitudeCommand`.

Tilt compensation: if `tilt_compensation = true`, throttle is divided by `cos(roll) × cos(pitch)` to maintain constant vertical thrust regardless of body tilt.

#### Methods

- **`CascadeController::new(mixer, profiler_z, profiler_xy, vel_loop_z, vel_loop_x, vel_loop_y, att_loop_roll, att_loop_pitch, att_loop_yaw) -> Self`**
  Constructor accepting all cascade components. Sets `max_tilt_rad = 0.15` and `tilt_compensation = true`.

### Function `make_cascade`

```rust
pub fn make_cascade(model: &dyn VehicleModel)
    -> CascadeController<SqrtProfiler, SqrtProfiler, PidLoop>
```

Factory function that creates a cascade controller with default PID parameters from a vehicle model. Automatically selects the mixer (`QuadrotorMixer` or `FixedWingMixer`) based on the model's equilibrium input type.

Default PID tuning:
- Z velocity: `PidLoop(0.3, 0.1, 0.0, 0.45, 0.45)`
- X/Y velocity: `PidLoop(0.4, 0.05, 0.0, 0.5, 0.35)`
- Roll/pitch angle: `PidLoop(4.0, 0.0, 0.2, 1.0, 1.0)`
- Yaw angle: `PidLoop(2.0, 0.1, 0.0, 0.5, 0.5)`

---

## 9. Module `lqr`

Linear-Quadratic Regulator (LQR) and Linear-Quadratic Integral (LQI) controller. Covers model linearisation, solving the Continuous Algebraic Riccati Equation (CARE), and controller design.

### 9.1. Sub-module `linearize`

Numerical linearisation of the vehicle model around an operating point.

#### Struct `LinearizedModel`

```rust
#[derive(Debug, Clone)]
pub struct LinearizedModel {
    pub a:  DMatrix<f64>,   // state dynamics matrix [13×13]
    pub b:  DMatrix<f64>,   // input matrix [13×m]
    pub x0: DVector<f64>,   // operating point — state vector
    pub u0: DVector<f64>,   // operating point — control vector
}
```

#### Fields

- **`a: DMatrix<f64>`** — state dynamics matrix A. Size 13×13 (13, not 12, because the quaternion has 4 components for 3 degrees of freedom).
- **`b: DMatrix<f64>`** — input matrix B. Size 13×m, where m = number of inputs (4 for a quadrotor).
- **`x0: DVector<f64>`** — state vector at the operating point (linearisation point).
- **`u0: DVector<f64>`** — control vector at the operating point (equilibrium).

#### Function `linearize`

```rust
pub fn linearize(
    model: &dyn VehicleModel,
    state0: &DroneState,
    input0: &KnownActuatorInput,
) -> LinearizedModel
```

Numerical linearisation of the nonlinear model around state `state0` and input `input0`. Computes matrices A and B via central finite differences with step ε = 1.49×10⁻⁸.

#### State conversion functions

- **`state_to_vec(state: &DroneState) -> DVector<f64>`**
  Converts `DroneState` to a 13D vector: `[x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]`.

- **`vec_to_state(vec: &DVector<f64>, template: &DroneState) -> DroneState`**
  Converts a 13D vector back to `DroneState`. `template` provides fields not contained in the vector (e.g. `actuator_state`).

- **`input_to_vec(input: &KnownActuatorInput) -> DVector<f64>`**
  Converts actuator input to a vector. For a quadrotor: `[FR, FL, RL, RR]`. For fixed-wing: `[throttle, aileron, elevator, rudder]`.

- **`vec_to_input(v: &DVector<f64>, template: &KnownActuatorInput) -> KnownActuatorInput`**
  Converts a vector back to `KnownActuatorInput`. `template` determines the output variant.

#### Discretisation functions

- **`discretize_euler(a: &DMatrix<f64>, b: &DMatrix<f64>, dt: f64) -> (DMatrix<f64>, DMatrix<f64>)`**
  Forward Euler discretisation: `Ad = I + A·dt`, `Bd = B·dt`. Cheap and accurate for small dt, but numerically unstable for large steps (Ad eigenvalues may exit the unit circle).

- **`discretize_implicit_euler(a: &DMatrix<f64>, b: &DMatrix<f64>, dt: f64) -> (DMatrix<f64>, DMatrix<f64>)`**
  Backward (implicit) Euler discretisation: `Ad = (I − A·dt)⁻¹`, `Bd = Ad · B·dt`. A-stable — Ad eigenvalues always remain inside the unit circle for any stable or marginally stable continuous system and any dt > 0. Safe for large prediction steps.

### 9.2. Sub-module `care`

Solving the Continuous Algebraic Riccati Equation (CARE) for LQR.

#### Enum `CareError`

```rust
#[derive(Debug, Error)]
pub enum CareError {
    WrongDimensionsA { rows: usize, cols: usize },
    WrongDimensionsB { expected: usize, got: usize },
    WrongDimensionsQ { n: usize, rows: usize, cols: usize },
    WrongDimensionsR { m: usize, rows: usize, cols: usize },
    SingularR,
    SingularLyapunov,
    NotConverged { max_iter: usize, residual: f64 },
}
```

#### Variants

- **`WrongDimensionsA`** — matrix A is not square.
- **`WrongDimensionsB`** — B has the wrong number of rows (must equal the dimension of A).
- **`WrongDimensionsQ`** — Q has wrong dimensions (must be n×n).
- **`WrongDimensionsR`** — R has wrong dimensions (must be m×m).
- **`SingularR`** — matrix R is singular (non-invertible). Diagonal elements must be positive.
- **`SingularLyapunov`** — Lyapunov system is singular (zero closed-loop eigenvalue persists after regularisation).
- **`NotConverged`** — CARE did not converge within `max_iter` Newton iterations. `residual` contains the final residual.

#### Struct `SolverParams`

```rust
#[derive(Debug, Clone)]
pub struct SolverParams {
    pub max_iter:  usize,
    pub tolerance: f64,
}
```

- **`max_iter: usize`** — maximum number of iterations. Default `1000`.
- **`tolerance: f64`** — convergence tolerance. Default `1e-8`.

#### Struct `RiccatiSolution`

```rust
#[derive(Debug, Clone)]
pub struct RiccatiSolution {
    pub p:             DMatrix<f64>,
    pub k:             DMatrix<f64>,
    pub flow_steps:    usize,
    pub newton_iters:  usize,
    pub care_residual: f64,
}
```

- **`p: DMatrix<f64>`** — solution matrix P of the CARE.
- **`k: DMatrix<f64>`** — gain matrix K = R⁻¹BᵀP.
- **`flow_steps: usize`** — number of Riccati flow RK4 steps (phase 1).
- **`newton_iters: usize`** — number of Newton-Kleinman iterations (phase 2). `0` means phase 1 was sufficient.
- **`care_residual: f64`** — final CARE residual.

#### Function `solve_care`

```rust
pub fn solve_care(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    q: &DMatrix<f64>,
    r: &DMatrix<f64>,
    params: &SolverParams,
) -> Result<RiccatiSolution, CareError>
```

Solves the Continuous Algebraic Riccati Equation: `AᵀP + PA − PBR⁻¹BᵀP + Q = 0`.

Two-phase algorithm:
1. **Phase 1**: Integrate the Riccati ODE with RK4 to obtain a good approximation of P.
2. **Phase 2**: Refine with Newton-Kleinman (iterative Lyapunov equation solving).

Automatically handles "dead" state directions — state directions decoupled at the operating point (e.g. a quaternion component in hover). Q weights are zeroed for these directions, giving a physically correct K = 0.

#### Helper functions

- **`build_q_diagonal(weights: &[f64]) -> DMatrix<f64>`**
  Builds a diagonal Q matrix from a weight vector. Result size: n×n where n = length of `weights`.

- **`build_r_diagonal(weights: &[f64]) -> DMatrix<f64>`**
  Builds a diagonal R matrix from a weight vector. Result size: m×m where m = length of `weights`.

### 9.3. Sub-module `lqr`

LQR controller — stabilisation around a fixed operating point.

#### Struct `LqrController`

```rust
pub struct LqrController {
    // private fields: k, x0, u0, input_template, u_limits
}
```

LQR controller designed offline for one operating point (equilibrium). Stabilises the drone around that point regardless of `FlightTarget`. Does not track arbitrary targets — use `LqiController` for tracking.

#### Methods

- **`LqrController::design(model: &dyn VehicleModel, trim_state: &DroneState, q_weights: &[f64], r_weights: &[f64], u_limits: Vec<(f64, f64)>) -> Result<Self, CareError>`**
  Designs an LQR controller around state `trim_state`:
  - `model` — vehicle model (used for linearisation and equilibrium input).
  - `trim_state` — hover/cruise state around which to linearise.
  - `q_weights` — diagonal Q weights (13 elements for a quadrotor: position, velocity, angular velocity, quaternion).
  - `r_weights` — diagonal R weights (4 elements for a quadrotor: one per motor).
  - `u_limits` — actuator input limits `[(min, max); m]`.
  Returns `Err(CareError)` if the CARE solver does not converge.

- **`compute_control(&self, state: &DroneState) -> DVector<f64>`**
  Computes the control vector: `u = u₀ − K·(x − x₀)`, clamped to `u_limits`.

Implements `Controller`: the `update()` method ignores `target` and `dt` — LQR always stabilises around the design point.

### 9.4. Sub-module `lqi`

LQI controller — LQR extended with 4 integral states to eliminate steady-state error.

#### Enum `LqiError`

```rust
#[derive(Debug, Error)]
pub enum LqiError {
    WrongCIntShape { n_integrals: usize, n_plant: usize, rows: usize, cols: usize },
    WrongQWeightsLen { expected: usize, n_plant: usize, n_integrals: usize, actual: usize },
    Care(CareError),
}
```

- **`WrongCIntShape`** — `c_int` matrix has wrong dimensions (expected 4×n_plant).
- **`WrongQWeightsLen`** — `q_weights` has wrong length (expected n_plant + 4).
- **`Care(CareError)`** — CARE solver error.

#### Struct `LqiController`

```rust
pub struct LqiController {
    // private fields: k, x0, u0, xi, input_template, u_limits
    pub xi_limits: [f64; 4],
}
```

Augmented state: `z = [δx (13D deviation from operating point); ξ (4D integrals)]` — 17D total.

The gain matrix K ∈ ℝ^(m×17) is computed once by CARE on the augmented system and does not change during operation.

#### Public field

- **`xi_limits: [f64; 4]`** — anti-windup limits for each integrator [m·s, m·s, m·s, rad·s]. Defaults: `[5.0, 5.0, 2.0, 2π]`.

#### Methods

- **`LqiController::design(model: &dyn VehicleModel, trim_state: &DroneState, c_int: DMatrix<f64>, q_weights: &[f64], r_weights: &[f64], u_limits: Vec<(f64, f64)>) -> Result<Self, LqiError>`**
  Designs an LQI controller:
  - `c_int` — output selection matrix (4×n_plant). Maps plant states to integrated outputs. For the standard quadrotor configuration, use `quadrotor_c_integral(n_plant)`.
  - `q_weights` — must have length `n_plant + 4` (17 for a quadrotor): indices 0..n_plant are state deviation weights, indices n_plant.. are integral weights [ξ_x, ξ_y, ξ_z, ξ_ψ]. Typical integral weights: 5–50.
  - Other parameters as in `LqrController::design`.

  Builds the augmented system:
  ```
  A_aug = [A  0]     B_aug = [B]
          [-C 0]              [0]
  ```
  and solves CARE on it.

- **`update_integrals(&mut self, state: &DroneState, target: &FlightTarget, dt: f64)`** *(private)*
  Updates integral states for active `FlightTarget` axes. Axes with `None` have frozen integrators (ξ̇ = 0). Anti-windup clamping applied to `xi_limits`.

Implements `Controller`:
- `update()` — updates integrals, computes `u = u₀ − K·z`, and returns actuator signals.
- `reset()` — zeros all 4 integrators.

#### Function `quadrotor_c_integral`

```rust
pub fn quadrotor_c_integral(n_plant: usize) -> DMatrix<f64>
```

Returns the standard selection matrix C ∈ ℝ^(4×n_plant) for a quadrotor:
- ξ_x ← x (state index 0)
- ξ_y ← y (state index 1)
- ξ_z ← z (state index 2)
- ξ_ψ ← 2·qz (state index 11; yaw linearisation around the identity quaternion: d(yaw)/d(qz)|_{q=I} = 2)

---

## 10. Module `trajectory`

Time-varying trajectory generators for open-loop path tracking.

### Trait `Trajectory`

```rust
pub trait Trajectory: Send + Sync {
    fn target(&self, time_s: f64) -> FlightTarget;
}
```

#### Methods

- **`target(&self, time_s: f64) -> FlightTarget`**
  Computes the flight target at the given simulation time [s]. Called every simulation step.

### Struct `HoldTrajectory`

```rust
#[derive(Debug, Clone)]
pub struct HoldTrajectory {
    pub inner: FlightTarget,
}
```

Always returns the same target — a no-op wrapper for a constant setpoint.

#### Fields

- **`inner: FlightTarget`** — constant target returned for every time instant.

### Struct `WaypointTrajectory`

```rust
#[derive(Debug, Clone)]
pub struct WaypointTrajectory {
    // private: waypoints: Vec<(f64, FlightTarget)>
}
```

Piecewise-linear trajectory through timed waypoints.

#### Interpolation behaviour

- Waypoints: `Vec<(time_s, FlightTarget)>` sorted in ascending time order.
- **Before the first waypoint** → the first waypoint is held.
- **After the last waypoint** → the last waypoint is held.
- **Between waypoints** → linear interpolation of axes with `Some`. For each axis:
  - Both `Some` → linear interpolation (lerp).
  - Only one `Some` → that side's value is held.
  - Both `None` → result is `None`.

#### Methods

- **`WaypointTrajectory::new(wps: Vec<(f64, FlightTarget)>) -> Self`**
  Creates a trajectory from a list of `(time_s, FlightTarget)` pairs. The list is sorted by time. **Panics** if `wps` is empty.

### Struct `CircleTrajectory`

```rust
#[derive(Debug, Clone)]
pub struct CircleTrajectory {
    pub cx:         f64,
    pub cy:         f64,
    pub radius:     f64,
    pub omega:      f64,
    pub altitude_m: f64,
}
```

Circular orbit in the horizontal plane at a constant altitude.

#### Fields

- **`cx: f64`** — orbit centre X coordinate [m].
- **`cy: f64`** — orbit centre Y coordinate [m].
- **`radius: f64`** — orbit radius [m].
- **`omega: f64`** — angular velocity [rad/s]; positive = CCW (counter-clockwise).
- **`altitude_m: f64`** — constant flight altitude [m].

Position at time t: `x = cx + radius·cos(ω·t)`, `y = cy + radius·sin(ω·t)`, `z = altitude_m`.
