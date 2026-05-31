# drone-sim

A 6DOF flight simulator for a quadrotor (DJI Mini 3) and a fixed-wing aircraft (F-16A).
Written in Rust as a workspace with 7 library crates and 4 CLI tools.

## Quick Start

```bash
# Run SITL tests with all scenarios
cargo run --bin sitl-test

# Compare controllers (Cascade PID, LQR, LQI)
cargo run --bin sitl-compare

# Monte Carlo — 100 iterations with perturbed initial conditions
cargo run --bin monte-carlo -- -s scenarios/step_response.toml --runs 100

# Validate the model against real DJI telemetry
cargo run --bin telem-analyze -- data/DJI_0001.srt --plot
```

## Project Structure

```
crates/
  drone-model/       Physical model — 6DOF dynamics, vehicles, motors, atmosphere
  drone-control/     Controllers — Cascade PID, LQR, LQI, mixers, trajectories
  drone-sim/         Simulation engine — integrators (Euler, RK4), runner
  drone-sitl/        SITL test harness — scenarios, metrics, comparisons, Monte Carlo
  drone-telemetry/   DJI telemetry parser (.srt files)
  drone-analysis/    Model validation vs. real telemetry
  drone-plot/        PNG chart generation (plotters)

bin/
  sitl-test/         Run SITL scenarios
  sitl-compare/      Side-by-side controller comparison
  monte-carlo/       Monte Carlo simulation with perturbed initial conditions
  telem-analyze/     Validate the model against DJI drone data
```

## Documentation

Detailed documentation for each crate — structs, traits, enums, functions, usage examples:

| Document | Contents |
|----------|----------|
| [docs/architecture.md](docs/architecture.md) | Architecture, dependency graph, data flow, conventions |
| [docs/drone-model.md](docs/drone-model.md) | `DroneState`, `VehicleModel`, quadrotor, F-16, atmosphere, math |
| [docs/drone-control.md](docs/drone-control.md) | `Controller`, Cascade PID, LQR/LQI, CARE solver, mixers, trajectories |
| [docs/drone-sim.md](docs/drone-sim.md) | Integrators (Euler/RK4), `SimFrame`, `SimConfig`, runner |
| [docs/drone-sitl.md](docs/drone-sitl.md) | TOML scenarios, metrics, comparisons, Monte Carlo, disturbances |
| [docs/drone-telemetry-and-analysis.md](docs/drone-telemetry-and-analysis.md) | SRT parser, GPS→ENU normalisation, model validation |
| [docs/cli.md](docs/cli.md) | CLI tools — flags, examples, output format |

## Tests

```bash
cargo test              # all unit tests (134 tests)
cargo test -p drone-model    # physical model only
cargo test -p drone-control  # controllers only
```
