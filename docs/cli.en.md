# Command-Line Tools (CLI)

## Overview

The `drone-sim` project ships four executable programs (binary crates in `bin/`):

| Tool | Description |
|------|-------------|
| `sitl-test` | Runs SITL scenarios and reports pass/fail for each one. |
| `sitl-compare` | Compares several flight controllers side-by-side on the same set of scenarios. |
| `monte-carlo` | Monte Carlo simulation — runs a scenario multiple times with perturbed initial conditions. |
| `telem-analyze` | Validates the physical model against real DJI telemetry (SRT files). |

All programs are built in the standard way:

```sh path=null start=null
cargo build -p sitl-test -p sitl-compare -p monte-carlo -p telem-analyze
```

---

## sitl-test

Runs a set of SITL (Software-In-The-Loop) scenarios with a chosen controller and checks whether the flight metrics satisfy the defined acceptance criteria.

### CLI Flags

```text path=null start=null
sitl-test [OPTIONS]
```

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--scenarios-dir <PATH>` | `PathBuf` | `scenarios` | Directory containing TOML scenario files. |
| `--controller <KIND>` | `ControllerKind` | `cascade` | Controller for quadrotor scenarios. F-16 scenarios **always** use a built-in LQR regardless of this flag. |
| `--config <PATH>`, `-c` | `PathBuf` (optional) | none | TOML file with controller parameters. When provided, overrides `--controller` — the `type` field in the file determines the controller kind. |
| `--plot` | `bool` | `false` | Generate step-response PNG charts to `target/` for each scenario. |

### `ControllerKind` Enum

Selects the controller kind for quadrotor scenarios:

```rust path=null start=null
enum ControllerKind {
    /// Cascade PID (position → velocity → attitude). Default.
    Cascade,
    /// Linear-Quadratic Regulator — stabilises around an operating point.
    Lqr,
    /// LQR with integral action — tracks a commanded setpoint.
    Lqi,
}
```

### F-16 Handling

Scenarios with `vehicle = "f16"` use a dedicated `f16_lqr_factory` that:

1. **Warms up the jet engine** — 1000 steps of 0.01 s each (10 s >> 5τ = 0.5 s), to avoid CARE solver divergence at zero thrust.
2. **Builds a trim state** — level flight at sea level, V = 200 m/s, angle of attack α = 5°.
3. **Designs an LQR controller** with Q weights (13 states) and R weights (4 actuators: throttle, aileron, elevator, rudder).

The `--controller` flag is ignored for F-16 scenarios.

### Usage Examples

Basic run with all scenarios and default cascade controller:

```sh path=null start=null
cargo run -p sitl-test
```

With LQR controller:

```sh path=null start=null
cargo run -p sitl-test -- --controller lqr
```

With a controller config file:

```sh path=null start=null
cargo run -p sitl-test -- --config controllers/cascade.toml
```

With chart generation:

```sh path=null start=null
cargo run -p sitl-test -- --controller lqi --plot
```

### Output Format

For each scenario the program prints the result (PASS/FAIL) with metric values and their thresholds. A summary is printed at the end:

```text path=null start=null
═══════════════════════════════
  Results: 8 PASS, 1 FAIL
═══════════════════════════════
```

A Markdown report is also generated at `target/sitl_report_YYYY-MM-DD_HH-MM.md` containing a table of all scenarios, results, metric values, and links to charts (if `--plot`).

The program exits with code `1` if any scenario failed.

---

## sitl-compare

Compares several flight controllers on the same set of scenarios, generating metric tables, CSV files, and charts.

### CLI Flags

```text path=null start=null
sitl-compare [OPTIONS]
```

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--config <PATH>`, `-c` | `PathBuf` (optional) | none | TOML file with the list of controllers to compare (`CompareConfig` format). When omitted, a default set of 4 controllers is used. |
| `--scenarios <PATHS>` | `Vec<PathBuf>` (optional, comma-separated) | `scenarios/step_response.toml`, `scenarios/disturbance_rejection.toml`, `scenarios/turbulence_comparison.toml` | TOML scenario files to run. |

### Default Controller Set

When `--config` is not provided, four controllers are compared:

1. **Cascade-PID** — cascade PID with default parameters
2. **LQR-R=0.01** — LQR with default R weights (aggressive)
3. **LQR-R=1.0** — LQR with R = 1.0 per motor (gentle)
4. **LQI** — LQI with default parameters

### TOML Config Format (`CompareConfig`)

The file defines a `[[controllers]]` array where each entry has a `name` and a nested `[controllers.config]` table:

```toml path=null start=null
[[controllers]]
name = "Cascade-default"
[controllers.config]
type = "cascade"
max_tilt_deg = 8.6
[controllers.config.vel_z]
kp = 0.3  ki = 0.1  kd = 0.0  integral_limit = 0.45  output_limit = 0.45

[[controllers]]
name = "LQR-aggressive"
[controllers.config]
type = "lqr"
trim_z_m = 5.0
q_weights = [1.0, 1.0, 100.0, 0.5, 0.5, 5.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0]
```

`NamedController` struct:

```rust path=null start=null
struct NamedController {
    /// Label displayed in the comparison table.
    name: String,
    /// Controller configuration (same format as files in controllers/).
    config: ControllerConfig,
}
```

### Output

For each scenario the program generates:

- **Comparison table** on stdout with columns: Controller, RMS Z [m], OS [%] (overshoot), ST [s] (settling time), RT [s] (rise time), Energy
- **Trajectory CSV** — `target/{scenario}_trajectories.csv`
- **Metrics CSV** — `target/{scenario}_metrics.csv`
- **PNG charts** — `target/{scenario}_trajectories.png` and `target/{scenario}_metrics.png`
- **Markdown report** — `target/report_YYYY-MM-DD_HH-MM.md`

### Usage Examples

Compare default controllers on default scenarios:

```sh path=null start=null
cargo run -p sitl-compare
```

With a custom controller config:

```sh path=null start=null
cargo run -p sitl-compare -- --config controllers/compare.toml
```

With a selected scenario:

```sh path=null start=null
cargo run -p sitl-compare -- --scenarios scenarios/step_response.toml,scenarios/hover_stability.toml
```

---

## monte-carlo

Runs a SITL scenario multiple times with randomly perturbed initial conditions (position, velocity) and aggregates metric statistics. Individual runs are executed in parallel.

### CLI Flags

```text path=null start=null
monte-carlo [OPTIONS] -s <PATH>
```

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--scenario <PATH>`, `-s` | `PathBuf` | (required) | Path to the TOML scenario file. |
| `--runs <N>` | `usize` | `100` | Number of independent simulation runs. |
| `--pos-noise <σ>` | `f64` | `0.5` | Standard deviation of initial position noise [m]. |
| `--vel-noise <σ>` | `f64` | `0.1` | Standard deviation of initial velocity noise [m/s]. |
| `--seed <SEED>` | `u64` | `42` | PRNG seed (for reproducible results). |
| `--controller <KIND>` | `ControllerKind` | `cascade` | Controller kind (`cascade`, `lqr`, `lqi`). |
| `--config <PATH>`, `-c` | `PathBuf` (optional) | none | TOML file with controller parameters. When provided, overrides `--controller`. |

### Behaviour

1. Loads the TOML scenario and creates a `QuadrotorModel::mini3()`.
2. For each of N runs, adds Gaussian noise to the initial position and velocity.
3. Executes all runs in parallel.
4. Aggregates statistics (mean, standard deviation, min, max) for each metric.

### Output

- **Statistics table** on stdout
- **CSV file** — `target/{scenario}_mc.csv`
- **PNG chart** — `target/{scenario}_mc.png`

### Usage Examples

Basic run (100 runs, default noise):

```sh path=null start=null
cargo run -p monte-carlo -- -s scenarios/step_response.toml
```

500 runs with larger noise and LQI controller:

```sh path=null start=null
cargo run -p monte-carlo -- \
    -s scenarios/step_response.toml \
    --runs 500 \
    --pos-noise 1.0 \
    --vel-noise 0.3 \
    --controller lqi
```

With a controller config file and fixed seed:

```sh path=null start=null
cargo run -p monte-carlo -- \
    -s scenarios/disturbance_rejection.toml \
    -c controllers/lqr.toml \
    --seed 12345 \
    --runs 200
```

---

## telem-analyze

Validates the drone's physical model against real DJI telemetry. The program parses an SRT subtitle file, normalises GPS points into ENU coordinates, runs an open-loop simulation of the Mini 3 model, and compares the trajectories.

### CLI Flags

```text path=null start=null
telem-analyze [OPTIONS] [FILE]
```

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `<file>` (positional) | `PathBuf` | `data/DJI_0001.srt` | DJI `.srt` file to analyse. |
| `--dt-s <DT>`, `-d` | `f64` (optional) | mean SRT frame interval | Simulation time step [s]. Defaults to the reciprocal of the telemetry frame rate. |
| `--threshold-m <THRESH>` | `f64` | `VALID_POSITION_THRESHOLD_M` | Position error threshold [m] for the "model valid until t" metric. A point is considered valid when `|pos_error| < threshold`. |
| `--save-csv`, `-o` | `bool` | `false` | Save a point-by-point comparison table to a CSV file next to the input file. |
| `--plot` | `bool` | `false` | Generate a validation PNG chart in `target/`. |

### Processing Pipeline

1. **SRT parsing** — `parse_file()` extracts telemetry frames from the DJI SRT file.
2. **GPS normalisation** — `normalize()` converts GPS coordinates to a local ENU frame, computes flight duration, maximum altitude, and speed.
3. **Open-loop simulation** — `QuadrotorModel::mini3()` is run with time step `dt` (defaulting to the telemetry frame rate, fallback 30 fps = 0.033 s).
4. **Alignment and comparison** — `validate_model()` compares model and telemetry trajectories and computes error metrics.
5. **Report** — results are printed to stdout with position and velocity metrics.

### Output Format

Printed to stdout:

- Number of parsed SRT frames
- Flight duration and number of GPS points
- Maximum altitude [m] and speed [m/s, km/h]
- Validation metrics (from `report.print()`)

Optionally:
- **CSV** — `{input_file}.validation.csv` (with `--save-csv`)
- **PNG** — chart in `target/` (with `--plot`)

### Usage Examples

Analyse the default telemetry file:

```sh path=null start=null
cargo run -p telem-analyze
```

Analyse a specific file with CSV output and chart:

```sh path=null start=null
cargo run -p telem-analyze -- data/DJI_0042.srt --save-csv --plot
```

With custom time step and threshold:

```sh path=null start=null
cargo run -p telem-analyze -- data/DJI_0042.srt -d 0.02 --threshold-m 5.0
```

---

## TOML Configuration Examples

### Cascade PID Controller (`cascade.toml`)

Three-level cascade PID: position → velocity → attitude → motor commands.

```toml path=null start=null
type = "cascade"

# Maximum tilt angle for XY control [deg].
max_tilt_deg = 8.6

# Vertical velocity loop: vz error → delta throttle
[vel_z]
kp             = 0.3
ki             = 0.1
kd             = 0.0
integral_limit = 0.45
output_limit   = 0.45

# Horizontal velocity loops (shared config for X and Y): vx/vy error → target pitch/roll
[vel_xy]
kp             = 0.4
ki             = 0.05
kd             = 0.0
integral_limit = 0.5
output_limit   = 0.35

# Attitude loops (shared config for roll and pitch): angle error → delta motor commands
[att]
kp             = 4.0
ki             = 0.0
kd             = 0.2
integral_limit = 1.0
output_limit   = 1.0

# Yaw loop
[att_yaw]
kp             = 2.0
ki             = 0.1
kd             = 0.0
integral_limit = 0.5
output_limit   = 0.5
```

`PidConfig` parameters for each loop:

- `kp` — proportional gain
- `ki` — integral gain
- `kd` — derivative gain
- `integral_limit` — anti-windup clamp on the integral accumulator
- `output_limit` — clamp on the loop output

### LQR Controller (`lqr.toml`)

Linear-Quadratic Regulator. CARE solver runs once per scenario. Stabilises the trim point, does **not** track setpoints.

```toml path=null start=null
type = "lqr"

# Trim altitude for linearisation [m].
trim_z_m = 5.0

# Q weight vector — 13 elements (quadrotor state vector):
#   [x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]
# Higher z/vz weights improve altitude tracking;
# higher quaternion weights keep the drone level.
q_weights = [
  1.0,  1.0,  50.0,
  0.5,  0.5,   5.0,
  2.0,  2.0,   2.0,
  20.0, 20.0, 20.0, 20.0,
]

# R weight vector — 4 elements (one per motor).
# Larger values → smoother, less aggressive control.
r_weights = [0.01, 0.01, 0.01, 0.01]
```

### LQI Controller (`lqi.toml`)

LQR extended with four integral states `[ξ_x, ξ_y, ξ_z, ξ_ψ]`, eliminating steady-state position/yaw error under constant disturbances (wind, drag, battery voltage drop).

```toml path=null start=null
type = "lqi"

trim_z_m = 5.0

# Q weight vector — 17 elements:
#   13 plant states (same as LQR) + 4 integral states [ξ_x, ξ_y, ξ_z, ξ_ψ]
q_weights = [
  # 13 plant weights
  1.0,  1.0,  50.0,
  0.5,  0.5,   5.0,
  2.0,  2.0,   2.0,
  20.0, 20.0, 20.0, 20.0,
  # 4 integral weights
  5.0,  5.0,  30.0,  2.0,
]

r_weights = [0.01, 0.01, 0.01, 0.01]

# Anti-windup limits for each integrator [m·s, m·s, m·s, rad·s].
# Optional — defaults to [30, 30, 30, 2π].
# xi_limits = [30.0, 30.0, 30.0, 6.2832]
```

### Scenario TOML File

Each scenario defines initial conditions, goal, duration, and acceptance criteria:

```toml path=null start=null
name = "step_response"
description = "Step from 0m to 5m — test transition characteristics"
duration_s = 15.0
dt_s = 0.005

[target]
z = 5.0

[initial]
position = [0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0]

[[assertions]]
metric = { position_rms_axis = "z" }
max = 2.0

[[assertions]]
metric = "settling_time_s"
max = 8.0

[[assertions]]
metric = "overshoot_percent"
max = 30.0
```

F-16 scenarios additionally set `vehicle = "f16"` and may specify an initial attitude in degrees (`attitude_deg = [roll, pitch, yaw]`).
