# `drone-model` — Reference Documentation

## 1. Overview

The `drone-model` crate provides physical models for flying vehicles (quadrotor, F-16 aircraft) and the infrastructure needed for 6-DOF (six degrees of freedom) flight simulation. It includes:

- Drone state representation (`DroneState`) with position, velocity, attitude (quaternion), and actuator state.
- Time step with validation (`TimeStep`).
- Indexed array of four quadrotor motors (`Motor`, `MotorArray<T>`).
- Vehicle model interfaces (`VehicleModel`, `AeroModel`) and shared force/moment structs.
- 6-DOF rigid body dynamics (`dynamics_6dof`).
- Full quadrotor model (DJI Mini 3 parameters) with rotors and aerodynamic drag.
- F-16A aircraft model with tabulated aerodynamics, jet engine, and a trim solver.
- Atmosphere models (ISA, constant density) and Euler ↔ quaternion conversions.

---

## 2. Module `state`

### `ActuatorState`

Internal actuator state stored in `DroneState`, making each state snapshot self-contained.

```rust
pub enum ActuatorState {
    QuadrotorMotors(MotorArray<f64>),
    FixedWingEngine {
        current_throttle: f64,
        current_thrust_n: f64,
    },
}
```

**Variants:**

- `QuadrotorMotors(MotorArray<f64>)` — angular speeds of the four motors [rad/s] in X-frame configuration.
- `FixedWingEngine { current_throttle, current_thrust_n }` — jet engine state: filtered throttle setting [0, 1] and resulting thrust in newtons.

### `DroneState`

Complete drone state at time *t*. All values in the world frame (ENU), except `angular_velocity` (body frame).

```rust
pub struct DroneState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub actuator_state: Option<ActuatorState>,
}
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `position` | `Vector3<f64>` | Position [x, y, z] in metres; z-axis points up. |
| `velocity` | `Vector3<f64>` | Linear velocity [vx, vy, vz] in m/s, world frame. |
| `angular_velocity` | `Vector3<f64>` | Angular velocity [p, q, r] in rad/s, **body frame**. |
| `orientation` | `UnitQuaternion<f64>` | Attitude as a unit quaternion — rotation from world to body frame. |
| `actuator_state` | `Option<ActuatorState>` | Optional actuator state (quadrotor motors or jet engine). |

**Methods:**

#### `euler_angles(&self) -> EulerAngles`

Returns the attitude as Euler angles (ZYX convention). Intended for visualisation and comparison with DJI telemetry.

#### `on_ground() -> Self`

Constructor that creates a zero state (position, velocity, angular velocity = 0; attitude = identity; no actuator state). Used as the simulation starting point.

---

## 3. Module `time`

### `TimeStep`

Newtype wrapping the simulation time step (`f64`). Guarantees dt > 0.

```rust
pub struct TimeStep(f64);
```

**Methods:**

#### `new(dt: f64) -> Result<Self, TimeStepError>`

Creates a time step. Returns `Err(TimeStepError)` if `dt <= 0`.

- `dt` — time step in seconds.

#### `constant(dt: f64) -> Self`

Creates a time step, panicking if `dt <= 0`. Use only for compile-time constants.

#### `seconds(self) -> f64`

Returns the step value in seconds.

#### `half(self) -> Self`

Returns half the time step. Useful for RK2/RK4 integrators.

### `TimeStepError`

Error returned when attempting to create a time step with a value ≤ 0.

```rust
pub struct TimeStepError(f64);
```

Implements `Display` (message: "TimeStep must be positive, got {value}") and `Error`.

---

## 4. Module `motor`

### `Motor`

Enum identifying the four quadrotor motors in X-frame configuration.

```rust
pub enum Motor {
    FrontRight = 0,  // CW (clockwise)
    FrontLeft  = 1,  // CCW
    RearLeft   = 2,  // CW
    RearRight  = 3,  // CCW
}
```

**Variants (top-down view):**

- `FrontRight` (index 0) — front-right, rotates clockwise (CW).
- `FrontLeft` (index 1) — front-left, rotates counter-clockwise (CCW).
- `RearLeft` (index 2) — rear-left, CW.
- `RearRight` (index 3) — rear-right, CCW.

**Constants:**

- `ALL: [Motor; 4]` — array of all variants in index order.

**Methods:**

#### `is_clockwise(self) -> bool`

Returns `true` for CW motors (`FrontRight`, `RearLeft`), `false` for CCW. The rotation directions are paired so that yaw torque is zero in hover.

### `MotorArray<T>`

An array of four values indexed by `Motor` variants. Generic — can store speeds (`f64`), forces, torques, etc.

```rust
pub struct MotorArray<T>([T; 4]);
```

**Methods (available for all `T`):**

#### `new(front_right: T, front_left: T, rear_left: T, rear_right: T) -> Self`

Creates an array with values in the order: FrontRight, FrontLeft, RearLeft, RearRight.

#### `iter(&self) -> impl Iterator<Item = (Motor, &T)>`

Iterator yielding `(Motor, &T)` pairs for all four motors.

**Methods (require `T: Copy`):**

#### `uniform(value: T) -> Self`

Creates an array with the same value for every motor.

#### `map<U, F: Fn(T) -> U>(&self, f: F) -> MotorArray<U>`

Applies function `f` to each element, returning a new array.

#### `map_with_motor<U, F: Fn(Motor, T) -> U>(&self, f: F) -> MotorArray<U>`

Like `map`, but the function also receives the motor identifier.

#### `sum(self) -> T` (requires `T: Add<Output = T>`)

Sums the four elements.

**Trait implementations:**

- `Index<Motor>` / `IndexMut<Motor>` — element access via `arr[Motor::FrontRight]`.
- `From<[T; 4]>` / `Into<[T; 4]>` — conversion from/to a plain array.

---

## 5. Module `vehicle`

### `KnownActuatorInput`

Control input for a vehicle.

```rust
pub enum KnownActuatorInput {
    Quadrotor(MotorArray<f64>),
    FixedWing {
        throttle: f64,
        aileron: f64,
        elevator: f64,
        rudder: f64,
    },
}
```

**Variants:**

- `Quadrotor(MotorArray<f64>)` — motor speed command [ω₀, ω₁, ω₂, ω₃] in rad/s.
- `FixedWing { throttle, aileron, elevator, rudder }`:
  - `throttle` — throttle [0, 1].
  - `aileron` — ailerons (roll) [−1, 1].
  - `elevator` — elevator (pitch) [−1, 1].
  - `rudder` — rudder (yaw) [−1, 1].

### `ForcesAndMoments`

Forces and moments acting on the vehicle in the body frame.

```rust
pub struct ForcesAndMoments {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}
```

- `force` — resultant force [N] in the body frame.
- `torque` — resultant torque [N·m] in the body frame.

Implements `Add` (summing forces and moments) and `Default` (zero forces/moments).

#### `new(force: Vector3<f64>, torque: Vector3<f64>) -> Self`

Constructor.

### `StateDot`

State derivatives with respect to time — result of the dynamics function.

```rust
pub struct StateDot {
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub angular_acceleration: Vector3<f64>,
    pub orientation_dot: Quaternion<f64>,
}
```

- `velocity` — ṗ = v (velocity → position derivative).
- `acceleration` — v̇ = F/m + g (linear acceleration in the world frame).
- `angular_acceleration` — ω̇ = I⁻¹(τ − ω×Iω) (angular acceleration in the body frame).
- `orientation_dot` — q̇ = ½·q⊗ω (quaternion attitude derivative).

### Trait `AeroModel`

Interface for an aerodynamic model.

```rust
pub trait AeroModel: Send + Sync {
    fn compute(
        &self,
        state: &DroneState,
        input: &KnownActuatorInput,
        atmosphere: &dyn AtmosphereModel,
    ) -> ForcesAndMoments;
}
```

#### `compute(&self, state, input, atmosphere) -> ForcesAndMoments`

Computes aerodynamic forces and moments for the given state, control input, and atmospheric conditions. Result in the body frame.

### Trait `VehicleModel`

Main interface for a flying vehicle model.

```rust
pub trait VehicleModel: Send + Sync { ... }
```

**Required methods:**

#### `derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot`

Computes state derivatives (dstate/dt) for the given state and input. Pure function — no side effects.

#### `equilibrium_input(&self) -> KnownActuatorInput`

Returns the equilibrium control input (e.g. hover for a quadrotor, level flight for an aircraft).

#### `name(&self) -> &str`

Returns a human-readable model name (e.g. `"QuadrotorModel (X-frame)"`).

#### `actuator_count(&self) -> usize`

Number of actuators (4 for both the quadrotor and F-16).

#### `mass(&self) -> f64`

Vehicle mass [kg].

**Default implementations:**

#### `step_actuators(&self, state: &mut DroneState, input: &KnownActuatorInput, dt: TimeStep)`

Updates the actuator state (e.g. first-order motor filter). Default implementation: no-op.

#### `gravity(&self) -> f64`

Gravitational acceleration [m/s²]. Default: `9.80665`.

#### `clone_box(&self) -> Box<dyn VehicleModel>`

Clones the model onto the heap as a trait object. Default implementation panics — each concrete model should override it. Used by controller factories that need their own copy of the model for linearisation.

---

## 6. Module `vehicle::dynamics_6dof`

### `RigidBodyParams`

Rigid body parameters: mass and inertia tensor with its inverse.

```rust
pub struct RigidBodyParams {
    pub mass: f64,
    pub inertia: Matrix3<f64>,
    pub inertia_inv: Matrix3<f64>,
}
```

- `mass` — mass [kg].
- `inertia` — inertia tensor [kg·m²] (3×3 matrix).
- `inertia_inv` — inverse of the inertia tensor (computed in the constructor).

**Methods:**

#### `new(mass: f64, ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> Self`

Creates parameters with a full inertia tensor. Off-diagonal elements are negated (convention: the inertia matrix has negative products of inertia). Panics if the tensor is singular.

#### `symmetric(mass: f64, ixx: f64, iyy: f64, izz: f64) -> Self`

Creates parameters for a symmetric vehicle (Ixy = Ixz = Iyz = 0). Typical for quadrotors.

### `dynamics_6dof`

Main 6-DOF rigid body dynamics function.

```rust
pub fn dynamics_6dof(
    state: &DroneState,
    fm: &ForcesAndMoments,
    params: &RigidBodyParams,
    gravity: f64,
) -> StateDot
```

**Parameters:**

- `state` — current drone state.
- `fm` — forces and moments in the body frame.
- `params` — rigid body parameters (mass, inertia tensor).
- `gravity` — gravitational acceleration [m/s²].

**Returns:** `StateDot` — state derivatives with respect to time.

**Physics (three stages):**

1. **Translation** — body-frame forces transformed to the world frame via the attitude quaternion, gravity added (ENU: z-axis up):
   - `F_world = R(q) · F_body`
   - `a = F_world / m + [0, 0, -g]`

2. **Rotation** — Euler's equation: `I·ω̇ = τ − ω×(I·ω)`. The `ω×(I·ω)` term is the gyroscopic effect of the rigid body — rotation changes the direction of the angular momentum, generating an additional moment. Without this term the simulation would be unstable at high angular rates.

3. **Quaternion derivative** — `q̇ = ½·q⊗ω`, where ω is written as a quaternion with zero scalar part. After integration the quaternion must be renormalised (the algebraic operation does not preserve |q| = 1).

---

## 7. Module `vehicle::quadrotor`

### `QuadrotorParams`

Quadrotor physical constants — loaded from a TOML file or created via the constructor.

```rust
pub struct QuadrotorParams {
    pub mass: f64,
    pub arm_length: f64,
    pub k_thrust: f64,
    pub k_torque: f64,
    pub k_drag: f64,
    pub rigid_body: RigidBodyParams,
}
```

**Fields:**

- `mass` — mass [kg].
- `arm_length` — arm length (centre of mass to motor) [m].
- `k_thrust` — thrust coefficient: F = k_thrust · ω² [N·s²/rad²].
- `k_torque` — reaction torque coefficient: τ = k_torque · ω² [N·m·s²/rad²].
- `k_drag` — body aerodynamic drag coefficient: F_drag = k_drag · v² [kg/m]. Isotropic quadratic drag opposing the velocity vector. Terminal velocity: v_t = √(m·g / k_drag).
- `rigid_body` — rigid body parameters (`RigidBodyParams`).

**Methods:**

#### `new(mass, arm_length, k_thrust, k_torque, k_drag, ixx, iyy, izz) -> Self`

Constructor. Creates `RigidBodyParams::symmetric` from the given moments of inertia.

#### `mini3() -> Self`

Returns parameters matching the DJI Mini 3 (mass 0.249 kg, arm 0.085 m). Includes detailed derivation of moments of inertia from motor and body masses.

### `QuadrotorAero`

Quadrotor aerodynamic model. Implements the `AeroModel` trait.

```rust
pub struct QuadrotorAero {
    pub params: QuadrotorParams,
}
```

Computes:
- **Thrust** per motor: F = k_thrust · ω².
- **Reaction torque** per motor: τ = k_torque · ω².
- **Resultant force** (sum of thrusts along body z + aerodynamic drag).
- **Moments**: roll from left–right thrust difference, pitch from rear–front difference, yaw from CW–CCW torque difference.

### `QuadrotorModel`

Full quadrotor model — implements the `VehicleModel` trait.

```rust
pub struct QuadrotorModel {
    pub params: QuadrotorParams,
    pub rotors: QuadrotorRotors,
    pub aero: QuadrotorAero,
    pub atmosphere: Box<dyn AtmosphereModel>,
}
```

**Methods:**

#### `new(params: QuadrotorParams, rotors: QuadrotorRotors, atmosphere: Box<dyn AtmosphereModel>) -> Self`

General constructor.

#### `mini3() -> Self`

DJI Mini 3 model with ISA atmosphere and rotors at hover speed. Hover speed computed analytically: ω = √(m·g / (4·k_thrust)).

#### `mini3_simple() -> Self`

Simplified Mini 3 with constant air density and rotors starting from zero. Useful for quick tests.

**`VehicleModel` implementation:**

- `derivatives` — computes aerodynamic forces, adds rotor gyroscopic torque, calls `dynamics_6dof`.
- `step_actuators` — first-order filter on motor speeds: `ω_new = α·ω_cur + (1−α)·ω_cmd`, where `α = exp(−dt/τ)`. Clamps speed to `[min_speed, max_speed]`.
- `equilibrium_input` — hover: ω = √(m·g / (4·k_thrust)) for each motor.
- `name` → `"QuadrotorModel (X-frame)"`.
- `actuator_count` → `4`.
- `mass` → `params.mass`.
- `clone_box` — clones the full model (including atmosphere via `clone_box`).

### `RotorParams`

Rotor dynamics parameters.

```rust
pub struct RotorParams {
    pub time_constant_s: f64,
    pub rotor_inertia: f64,
    pub max_speed: f64,
    pub min_speed: f64,
}
```

- `time_constant_s` — first-order time constant [s] (how quickly the motor responds to a command).
- `rotor_inertia` — rotor moment of inertia [kg·m²].
- `max_speed` — maximum angular speed [rad/s].
- `min_speed` — minimum angular speed [rad/s].

#### `mini3() -> Self`

Mini 3 rotor parameters: τ = 0.04 s, J = 2.0e-5 kg·m², max 1120 rad/s.

### `QuadrotorRotors`

Manages the state of four rotors with first-order dynamics.

```rust
pub struct QuadrotorRotors {
    pub params: RotorParams,
    current_speeds: MotorArray<f64>,
}
```

**Methods:**

#### `new(params: RotorParams) -> Self`

Creates rotors with initial speeds = 0.

#### `mini3() -> Self`

Shorthand: `Self::new(RotorParams::mini3())`.

#### `at_hover(params: RotorParams, hover_speed: f64) -> Self`

Creates rotors with initial speeds set to the hover value.

#### `speeds(&self) -> &MotorArray<f64>`

Returns the current angular speeds.

#### `step(&mut self, commanded: &MotorArray<f64>, dt: TimeStep)`

Performs one rotor dynamics step (first-order filter). Clamps speeds to `[min_speed, max_speed]`.

#### `gyroscopic_torque(&self, aircraft_angular_velocity: &Vector3<f64>) -> Vector3<f64>`

Computes the rotor gyroscopic torque: `τ_gyro = ω_aircraft × h_rotors`, where `h_rotors = [0, 0, J_r · Σ(σ_i · ω_i)]`. CW motors have σ = +1, CCW have σ = −1. In symmetric hover (all ω equal) the total rotor angular momentum is zero (CW and CCW cancel).

### `body_drag`

```rust
pub fn body_drag(velocity_world: &Vector3<f64>, k_drag: f64) -> Vector3<f64>
```

Computes the body aerodynamic drag force in the world frame.

- `velocity_world` — velocity in the world frame [m/s].
- `k_drag` — drag coefficient [kg/m].
- **Returns:** drag force vector = −v̂ · k_drag · |v|² (opposing velocity, scales with speed squared). Returns zero for |v| < 1e-6.

---

## 8. Module `vehicle::fixed_wing::f16`

### `F16Params`

F-16A mass and inertia parameters.

```rust
pub struct F16Params {
    pub mass: f64,
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyz: f64,
}
```

- `mass` — mass [kg].
- `ixx`, `iyy`, `izz` — principal moments of inertia [kg·m²].
- `ixy`, `ixz`, `iyz` — products of inertia [kg·m²].

#### `f16a() -> Self`

F-16A parameters per NASA TP-1538 (mass 9295.44 kg, Ixz = 1331.4 kg·m²).

### `F16Model`

Full F-16 model — implements the `VehicleModel` trait.

```rust
pub struct F16Model {
    pub params: F16Params,
    pub geom: F16GeomParams,
    pub engine: Mutex<JetEngine>,
    pub rigid_body: RigidBodyParams,
    pub atmosphere: Box<dyn AtmosphereModel>,
}
```

**Methods:**

#### `new(params, geom, engine, atmosphere) -> Self`

General constructor. Computes `RigidBodyParams` from mass parameters.

#### `f16a() -> Self`

F-16A configuration: NASA parameters, F-16A geometry, F110 engine (dry), ISA atmosphere.

**`VehicleModel` implementation:**

- `derivatives` — computes `AeroState`, fetches current engine thrust, calls `compute_aero` and `dynamics_6dof`.
- `step_actuators` — updates the jet engine (`JetEngine::step`) and saves engine state to `DroneState::actuator_state`.
- `equilibrium_input` — approximate level-flight control: throttle 0.5, elevator −0.06.
- `name` → `"F-16A (NASA TP-1538)"`.
- `actuator_count` → `4`.
- `mass` → `params.mass`.
- `clone_box` — creates a new `F16Model::f16a()` instance (the `Mutex<JetEngine>` is not cloned — the new model starts with a cold engine).

### `F16GeomParams`

Wing geometry parameters.

```rust
pub struct F16GeomParams {
    pub wing_area: f64,
    pub wingspan: f64,
    pub mean_chord: f64,
}
```

- `wing_area` — wing reference area [m²].
- `wingspan` — wingspan [m].
- `mean_chord` — mean aerodynamic chord [m].

#### `f16a() -> Self`

F-16A geometry: S = 27.87 m², b = 9.45 m, c̄ = 3.45 m.

### `AeroState`

Aerodynamic state computed from `DroneState` and the atmosphere model.

```rust
pub struct AeroState {
    pub airspeed: f64,
    pub alpha_rad: f64,
    pub beta_rad: f64,
    pub mach: f64,
    pub qbar: f64,
    pub v_body: Vector3<f64>,
}
```

- `airspeed` — airspeed [m/s] (min 1.0).
- `alpha_rad` — angle of attack α [rad] (clamped to [−0.5, 0.785]).
- `beta_rad` — sideslip angle β [rad] (clamped to [−0.524, 0.524]).
- `mach` — Mach number [−].
- `qbar` — dynamic pressure q̄ = ½ρV² [Pa].
- `v_body` — velocity in the body frame [u, v, w] [m/s].

#### `compute(state: &DroneState, atmosphere: &dyn AtmosphereModel) -> Self`

Computes the aerodynamic state from world velocity, attitude, and atmosphere model.

#### `alpha_deg(&self) -> f64` / `beta_deg(&self) -> f64`

Convert angles to degrees.

### `JetEngineParams`

Jet engine parameters.

```rust
pub struct JetEngineParams {
    pub thrust_sl_max: f64,
    pub thrust_sl_idle: f64,
    pub time_constant_s: f64,
    pub altitude_exp: f64,
}
```

- `thrust_sl_max` — maximum sea-level thrust at Mach = 0 [N].
- `thrust_sl_idle` — idle sea-level thrust [N].
- `time_constant_s` — engine time constant [s].
- `altitude_exp` — thrust altitude scaling exponent: thrust(h) ≈ thrust_sl · (ρ(h)/ρ_sl)^exp.

#### `f110_dry() -> Self`

F110 dry thrust parameters: 76 300 N max, 4 500 N idle, τ = 0.5 s.

### `JetEngine`

Jet engine state with first-order dynamics.

```rust
pub struct JetEngine {
    pub params: JetEngineParams,
    pub current_throttle: f64,
    current_thrust_n: f64,
}
```

**Methods:**

#### `new(params: JetEngineParams) -> Self`

Constructor — engine starts at zero throttle and zero thrust.

#### `f110_dry() -> Self`

Shorthand: `Self::new(JetEngineParams::f110_dry())`.

#### `thrust(&self) -> f64`

Current engine thrust [N].

#### `step(&mut self, throttle_cmd: f64, altitude_m: f64, mach: f64, atmosphere: &dyn AtmosphereModel, dt: TimeStep)`

Performs one engine dynamics step:
1. First-order filter on throttle (clamped to [0, 1]).
2. Thrust computed with altitude scaling (density ratio) and Mach correction (quadratic factor).
3. Linear interpolation between idle and maximum thrust based on current throttle.

### `compute_aero`

```rust
pub fn compute_aero(
    aero: &AeroState,
    angular: &Vector3<f64>,
    input: &KnownActuatorInput,
    geom: &F16GeomParams,
    thrust_n: f64,
) -> ForcesAndMoments
```

Computes F-16 aerodynamic forces and moments from coefficient tables.

- `aero` — aerodynamic state (α, β, V, q̄).
- `angular` — angular velocity [p, q, r] in body frame [rad/s].
- `input` — control (`FixedWing` with aileron, elevator, rudder in [−1, 1]).
- `geom` — wing geometry.
- `thrust_n` — current engine thrust [N].

Control surfaces are internally converted to deflection angles: elevator ×25°, aileron ×21.5°, rudder ×30°.

Aerodynamic coefficients interpolated from 1D tables (α-break, β-break). Thrust decomposed into axial components by angle of attack.

### `TrimResult`

Successful trim result.

```rust
pub struct TrimResult {
    pub alpha_rad: f64,
    pub throttle: f64,
    pub elevator: f64,
    pub residual: f64,
}
```

- `alpha_rad` — trim angle of attack [rad].
- `throttle` — trim throttle [0, 1].
- `elevator` — trim elevator (normalised to [−1, 1]).
- `residual` — norm of derivative residuals at the trim point [m/s²].

### `TrimError`

Trim failure.

```rust
pub enum TrimError {
    NoConvergence { residual: f64, iters: usize },
}
```

- `NoConvergence` — optimisation did not reach an acceptable residual. Contains the residual value and iteration count.

### `find_trim`

```rust
pub fn find_trim(
    _model: &F16Model,
    speed_ms: f64,
    altitude_m: f64,
) -> Result<TrimResult, TrimError>
```

Finds the longitudinal trim point (level flight, wings level) for the given speed and altitude.

- `speed_ms` — flight speed [m/s].
- `altitude_m` — altitude [m].

Searches for `(throttle, elevator, alpha)` minimising the sum of squared linear and angular acceleration using the Nelder-Mead simplex method (up to 500 iterations). Internally creates fresh `F16Model::f16a()` instances with a warm engine.

---

## 9. Module `math::atmosphere`

### Trait `AtmosphereModel`

Atmosphere model interface.

```rust
pub trait AtmosphereModel: Send + Sync {
    fn density(&self, altitude_m: f64) -> f64;
    fn temperature(&self, altitude_m: f64) -> f64;
    fn speed_of_sound(&self, altitude_m: f64) -> f64;
    fn dynamic_pressure(&self, altitude_m: f64, speed_ms: f64) -> f64;
    fn mach(&self, altitude_m: f64, speed_ms: f64) -> f64;
    fn clone_box(&self) -> Box<dyn AtmosphereModel>;
}
```

**Required methods:**

- `density(altitude_m) -> f64` — air density [kg/m³].
- `temperature(altitude_m) -> f64` — temperature [K].
- `speed_of_sound(altitude_m) -> f64` — speed of sound [m/s].
- `clone_box() -> Box<dyn AtmosphereModel>` — heap clone (needed because vehicle models hold `Box<dyn AtmosphereModel>`).

**Default implementations:**

- `dynamic_pressure(altitude_m, speed_ms) -> f64` — dynamic pressure: q = ½ρV².
- `mach(altitude_m, speed_ms) -> f64` — Mach number: M = V / a. Returns 0 if speed of sound ≤ 0.

### `Isa`

International Standard Atmosphere (ISA) implementation.

```rust
pub struct Isa;
```

Models the troposphere (0–11 000 m) with a linear temperature lapse (6.5 K/km) and the stratosphere (above 11 000 m) at a constant 216.65 K.

Constants (`isa_constants` module): T₀ = 288.15 K, P₀ = 101 325 Pa, L = 0.0065 K/m, R = 287.05 J/(kg·K), g = 9.80665 m/s², γ = 1.4.

### `ConstantDensity`

Simplified atmosphere model with altitude-independent constant density.

```rust
pub struct ConstantDensity {
    pub density: f64,
    pub speed_of_sound: f64,
}
```

- `density` — constant density [kg/m³].
- `speed_of_sound` — constant speed of sound [m/s].

#### `sea_level() -> Self`

Sea-level conditions: ρ = 1.225 kg/m³, a = 340.29 m/s. Constant temperature 288.15 K.

---

## 10. Module `math::euler`

### `EulerAngles`

Euler angles (ZYX convention — yaw → pitch → roll).

```rust
pub struct EulerAngles {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}
```

- `roll` — rotation about the X axis [rad].
- `pitch` — rotation about the Y axis [rad].
- `yaw` — rotation about the Z axis [rad].

**Methods:**

#### `new(roll: f64, pitch: f64, yaw: f64) -> Self`

Constructor (values in radians).

#### `from_degrees(roll: f64, pitch: f64, yaw: f64) -> Self`

Constructor from degree values (converts to radians).

#### `to_degrees(&self) -> (f64, f64, f64)`

Returns a tuple (roll, pitch, yaw) in degrees.

### `quat_to_euler`

```rust
pub fn quat_to_euler(q: &UnitQuaternion<f64>) -> EulerAngles
```

Converts a unit quaternion to Euler angles (ZYX).

Formulae:
- roll = atan2(2(qw·qx + qy·qz), 1 − 2(qx² + qy²))
- pitch = asin(2(qw·qy − qz·qx)) — clamped to [−1, 1] to avoid NaN from numerical error
- yaw = atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))

**Note:** singularity (gimbal lock) at pitch = ±90°.

### `euler_to_quat`

```rust
pub fn euler_to_quat(e: &EulerAngles) -> UnitQuaternion<f64>
```

Converts Euler angles to a unit quaternion. Rotation: R = Rz(yaw) · Ry(pitch) · Rx(roll).

The round-trip `euler_to_quat(quat_to_euler(q))` recovers the original quaternion (to numerical precision) except near the pitch = ±90° singularity.
