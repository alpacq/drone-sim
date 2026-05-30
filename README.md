# drone-sim

Symulator lotu 6DOF dla quadrotora (DJI Mini 3) i samolotu stałopłatowego (F-16A).
Napisany w Rust jako workspace z 7 bibliotekami i 4 narzędziami CLI.

## Szybki start

```bash
# Uruchom testy SITL ze wszystkimi scenariuszami
cargo run --bin sitl-test

# Porównaj regulatory (Cascade PID, LQR, LQI)
cargo run --bin sitl-compare

# Monte Carlo — 100 iteracji z zaburzeniami warunków początkowych
cargo run --bin monte-carlo -- -s scenarios/step_response.toml --runs 100

# Walidacja modelu na rzeczywistej telemetrii DJI
cargo run --bin telem-analyze -- data/DJI_0001.srt --plot
```

## Struktura projektu

```
crates/
  drone-model/       Model fizyczny — dynamika 6DOF, pojazdy, silniki, atmosfera
  drone-control/     Regulatory — Cascade PID, LQR, LQI, miksery, trajektorie
  drone-sim/         Silnik symulacji — integratory (Euler, RK4), runner
  drone-sitl/        Harness testowy SITL — scenariusze, metryki, porównania, Monte Carlo
  drone-telemetry/   Parser telemetrii DJI (pliki .srt)
  drone-analysis/    Walidacja modelu vs. rzeczywista telemetria
  drone-plot/        Generowanie wykresów PNG (plotters)

bin/
  sitl-test/         Uruchamianie scenariuszy SITL
  sitl-compare/      Porównanie regulatorów side-by-side
  monte-carlo/       Symulacja Monte Carlo z zaburzonymi warunkami początkowymi
  telem-analyze/     Walidacja modelu na danych z drona DJI
```

## Dokumentacja

Szczegółowa dokumentacja każdego crate'a — structy, traity, enumy, funkcje, przykłady użycia:

| Dokument | Zawartość |
|----------|-----------|
| [docs/architecture.md](docs/architecture.md) | Architektura, graf zależności, przepływ danych, konwencje |
| [docs/drone-model.md](docs/drone-model.md) | `DroneState`, `VehicleModel`, quadrotor, F-16, atmosfera, matematyka |
| [docs/drone-control.md](docs/drone-control.md) | `Controller`, Cascade PID, LQR/LQI, CARE solver, miksery, trajektorie |
| [docs/drone-sim.md](docs/drone-sim.md) | Integratory (Euler/RK4), `SimFrame`, `SimConfig`, runner |
| [docs/drone-sitl.md](docs/drone-sitl.md) | Scenariusze TOML, metryki, porównania, Monte Carlo, zakłócenia |
| [docs/drone-telemetry-and-analysis.md](docs/drone-telemetry-and-analysis.md) | Parser SRT, normalizacja GPS→ENU, walidacja modelu |
| [docs/cli.md](docs/cli.md) | Narzędzia CLI — flagi, przykłady, format wyjścia |

## Testy

```bash
cargo test              # wszystkie testy jednostkowe (134 testy)
cargo test -p drone-model    # tylko model fizyczny
cargo test -p drone-control  # tylko regulatory
```
