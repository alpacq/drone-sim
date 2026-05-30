# `drone-sim` — Reference Documentation

## 1. Overview

The `drone-sim` crate is the flight simulation engine. It is vehicle-agnostic — it operates on the `VehicleModel` trait from `drone-model`, so the same simulation code drives quadrotors, fixed-wing aircraft, and any future configurations.

The crate consists of two modules:

- **`integrator`** — numerical integration methods (Euler, RK4).
- **`runner`** — simulation loop: controller → actuators → integration → frame recording.

Re-exports from `lib.rs`:

```rust
pub mod integrator;
pub mod runner;
```

---

## 2. Module `integrator`

Contains the `Integrator` trait, the `apply_dot` helper function, and two implementations: `Euler` and `RK4`.

### 2.1 Trait `Integrator`

```rust
pub trait Integrator: Send + Sync {
    fn step(
        &self,
        model: &dyn VehicleModel,
        state: &DroneState,
        input: &KnownActuatorInput,
        dt: TimeStep,
    ) -> DroneState;
}
```

Interface for a numerical integration method. The trait is object-safe (`dyn Integrator`), allowing the integration method to be selected at runtime.

The `Send + Sync` super-traits enable use of the integrator in multi-threaded contexts.

#### Method `step`

Performs a single integration step of length `dt`.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `model` | `&dyn VehicleModel` | Vehicle dynamics model (computes state derivatives). |
| `state` | `&DroneState` | Current drone state (position, velocity, attitude, angular velocity). |
| `input` | `&KnownActuatorInput` | Control signal applied to actuators (e.g. motor speeds). |
| `dt` | `TimeStep` | Simulation time step. |

**Returns:** `DroneState` — new drone state after `dt`.

---

### 2.2 Function `apply_dot`

```rust
pub fn apply_dot(state: &DroneState, dot: &StateDot, dt: TimeStep) -> DroneState;
```

Applies derivatives (`StateDot`) to the current state, advancing it by `dt`. Used internally by `Euler` and `RK4`.

**Operations:**

- **Position:** `position += velocity * dt`
- **Velocity:** `velocity += acceleration * dt`
- **Angular velocity:** `angular_velocity += angular_acceleration * dt`
- **Attitude (quaternion):** `q += orientation_dot * dt`, then **renormalisation** via `UnitQuaternion::from_quaternion`. Renormalisation is required because adding the derivative to a unit quaternion arithmetically violates `|q| = 1`. Without this step, numerical drift would accumulate with each time step, corrupting the attitude.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `state` | `&DroneState` | Current drone state. |
| `dot` | `&StateDot` | State derivatives (velocity, acceleration, angular acceleration, attitude derivative). |
| `dt` | `TimeStep` | Time step. |

**Returns:** `DroneState` — state advanced by `dt`. The `actuator_state` field is copied from the input state unchanged.

---

### 2.3 Struct `Euler`

```rust
pub struct Euler;
```

Euler method — the simplest numerical integration method. Accuracy is **O(dt)** (global error grows linearly with the time step).

Useful for comparisons with RK4 and stability testing. For simulations requiring higher accuracy, use `RK4`.

Implements `Integrator`. The `step` method evaluates derivatives once and then calls `apply_dot`.

---

### 2.4 Struct `RK4`

```rust
pub struct RK4;
```

Fourth-order Runge-Kutta method. Accuracy is **O(dt⁴)** — the standard choice for flight simulators.

The algorithm computes four derivative estimates (k1–k4), combines them with a weighted average (1:2:2:1)/6, and applies the resulting derivative via `apply_dot`.

Implements `Integrator`. Steps:

1. `k1` — derivatives at the current state.
2. `k2` — derivatives at the state shifted by `k1 * dt/2`.
3. `k3` — derivatives at the state shifted by `k2 * dt/2`.
4. `k4` — derivatives at the state shifted by `k3 * dt`.
5. Weighted average: `(k1 + 2·k2 + 2·k3 + k4) / 6`.
6. `apply_dot` with the resulting derivative and the full `dt`.

---

## 3. Module `runner`

Contains the simulation loop and configuration structures.

### 3.1 Struct `SimFrame`

```rust
#[derive(Debug, Clone)]
pub struct SimFrame {
    pub time: f64,
    pub state: DroneState,
}
```

A single recorded simulation frame.

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `time` | `f64` | Simulation time in seconds from start. |
| `state` | `DroneState` | Full drone state at this moment. |

---

### 3.2 Struct `SimConfig`

```rust
pub struct SimConfig {
    pub dt: TimeStep,
    pub duration: f64,
}
```

Configuration for a simulation run.

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `dt` | `TimeStep` | Fixed simulation time step. |
| `duration` | `f64` | Total simulation duration in seconds. |

The number of steps is computed as `ceil(duration / dt)`.

---

### 3.3 Function `run`

```rust
pub fn run(
    initial_state: DroneState,
    model: &dyn VehicleModel,
    config: &SimConfig,
    integrator: &dyn Integrator,
    mut controller: impl FnMut(&DroneState, TimeStep) -> KnownActuatorInput,
) -> Vec<SimFrame>;
```

Main simulation function. Executes an open-loop run and returns the full frame history.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `initial_state` | `DroneState` | Initial drone state (position, velocity, attitude). |
| `model` | `&dyn VehicleModel` | Vehicle dynamics model. |
| `config` | `&SimConfig` | Simulation configuration (time step, duration). |
| `integrator` | `&dyn Integrator` | Integration method (e.g. `&RK4`). |
| `controller` | `impl FnMut(&DroneState, TimeStep) -> KnownActuatorInput` | Controller closure — given the current state and `dt`, returns the actuator input. The signature matches `Controller::update`, making it easy to adapt a `Controller` trait implementation to this interface. |

**Returns:** `Vec<SimFrame>` — vector of frames from `t = 0` (initial state) to `t ≈ duration`. Contains `steps + 1` elements (including the initial frame).

**One iteration of the loop:**

1. **Controller** — call the closure `controller(&state, dt)` → `KnownActuatorInput`.
2. **Actuators** — `model.step_actuators(&mut state, &input, dt)` updates the internal actuator state (e.g. motor dynamics).
3. **Integration** — `integrator.step(model, &state, &input, dt)` computes the new drone state.
4. **Frame recording** — the new state together with the current time is appended to the result vector.
