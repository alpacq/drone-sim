# `drone-sitl` — dokumentacja referencyjna

## 1. Przegląd

Crate `drone-sitl` dostarcza środowisko testowe **SITL** (Software-In-The-Loop) dla symulatora lotu dronów. Umożliwia:

- definiowanie scenariuszy testowych w plikach TOML (cel lotu, trajektoria, zakłócenia, asercje),
- konfigurowanie i porównywanie regulatorów (Cascade-PID, LQR, LQI),
- uruchamianie symulacji z zakłóceniami (porywy wiatru, turbulencje, awaria silnika),
- obliczanie metryk jakości sterowania (RMS, overshoot, czas ustalania, energia sterowania itd.),
- generowanie raportów i eksport do CSV,
- analizę Monte Carlo z równoległym wykonaniem (Rayon).

Moduły publiczne:

- `scenario` — definicja scenariusza testowego
- `controller_config` — konfiguracja regulatorów
- `runner` — pętla symulacji SITL
- `disturbance` — modele zakłóceń
- `metrics` — funkcje metryk
- `report` — raport ze scenariusza
- `comparison` — porównanie regulatorów
- `monte_carlo` — analiza Monte Carlo

Re-eksporty z `lib.rs`:

```rust path=null start=null
pub use controller_config::{CascadeConfig, ControllerConfig, LqiConfig, LqrConfig, PidConfig};
pub use monte_carlo::{MonteCarloConfig, MonteCarloReport};
```

---

## 2. Moduł `scenario`

Moduł odpowiada za deserializację scenariuszy testowych z formatu TOML.

### `Scenario`

Główna struktura definiująca pojedynczy scenariusz SITL.

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

Pola:

- `name` — nazwa scenariusza (wyświetlana w raportach).
- `description` — opcjonalny opis tekstowy.
- `duration_s` — czas trwania symulacji [s].
- `dt_s` — krok czasowy symulacji [s].
- `initial` — warunki początkowe (pozycja, prędkość, orientacja).
- `vehicle` — model pojazdu do symulacji (domyślnie `QuadrotorMini3`).
- `target` — statyczny punkt docelowy lotu.
- `trajectory` — opcjonalna trajektoria czasowa; gdy obecna, nadpisuje statyczny `target`.
- `disturbances` — lista konfiguracji zakłóceń (poryw wiatru, turbulencje, awaria silnika).
- `assertions` — lista asercji — warunków zaliczenia scenariusza.

#### `Scenario::from_file`

```rust path=null start=null
pub fn from_file(path: &std::path::Path) -> Result<Self, ScenarioError>
```

Wczytuje scenariusz z pliku TOML. Zwraca `ScenarioError::Io` przy błędach I/O lub `ScenarioError::Toml` przy błędach parsowania.

#### `FromStr` impl

```rust path=null start=null
impl std::str::FromStr for Scenario {
    type Err = ScenarioError;
    fn from_str(s: &str) -> Result<Self, ScenarioError>;
}
```

Parsuje scenariusz bezpośrednio z ciągu znaków TOML. Pozwala na użycie `s.parse::<Scenario>()`.

### `ScenarioError`

```rust path=null start=null
pub enum ScenarioError {
    Io(std::io::Error),
    Toml(toml::de::Error),
}
```

- `Io` — błąd odczytu pliku (brak pliku, brak uprawnień).
- `Toml` — błąd składni TOML.

### `VehicleKind`

Enum określający model pojazdu do symulacji.

```rust path=null start=null
pub enum VehicleKind {
    QuadrotorMini3,
    QuadrotorMini3Simple,
    F16,
}
```

Warianty:

- `QuadrotorMini3` *(domyślny)* — quadrotor DJI Mini 3 z pełnym modelem (atmosfera ISA, dynamika silników).
- `QuadrotorMini3Simple` — uproszczony DJI Mini 3 (stała gęstość atmosfery, szybsza linearyzacja).
- `F16` — odrzutowiec F-16A (model aerodynamiczny NASA TP-1538, silnik F110).

W TOML: `vehicle = "quadrotor_mini3"`, `"quadrotor_mini3_simple"`, `"f16"`.

### `ScenarioTarget`

Punkt docelowy lotu w scenariuszu.

```rust path=null start=null
pub struct ScenarioTarget {
    pub z: f64,
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub yaw: Option<f64>,
}
```

Pola:

- `z` — docelowa wysokość [m] — **wymagane**.
- `x` — docelowa pozycja X [m] — opcjonalnie (domyślnie brak kontroli osi X).
- `y` — docelowa pozycja Y [m] — opcjonalnie.
- `yaw` — docelowy kąt odchylenia [rad] — opcjonalnie.

### `InitialConditions`

Warunki początkowe symulacji.

```rust path=null start=null
pub struct InitialConditions {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub attitude_deg: [f64; 3],
}
```

Pola:

- `position` — pozycja początkowa `[x, y, z]` [m] (domyślnie `[0, 0, 0]`).
- `velocity` — prędkość początkowa `[vx, vy, vz]` [m/s] (domyślnie `[0, 0, 0]`).
- `attitude_deg` — orientacja początkowa `[roll, pitch, yaw]` [°] (domyślnie `[0, 0, 0]`). Wymagane np. dla F-16, który startuje z kątem natarcia α = 5°.

### `Assertion`

Pojedyncza asercja (warunek zaliczenia) scenariusza.

```rust path=null start=null
pub struct Assertion {
    pub metric: MetricKind,
    pub max: f64,
}
```

Pola:

- `metric` — rodzaj metryki do sprawdzenia.
- `max` — maksymalna dozwolona wartość metryki. Asercja jest spełniona, gdy `wartość ≤ max`.

### `MetricKind`

Enum określający rodzaj metryki jakości sterowania. Używany w asercjach scenariuszy.

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

Warianty:

- `PositionRms3d` — RMS błędu pozycji 3D.
- `PositionRmsAxis(Axis)` — RMS błędu pozycji wzdłuż jednej osi (X, Y lub Z).
- `PositionMaxError3d` — maksymalny błąd pozycji 3D.
- `PositionMaxErrorAxis(Axis)` — maksymalny błąd pozycji wzdłuż jednej osi.
- `VelocityRms3d` — RMS prędkości 3D (odchylenie od spoczynku).
- `VelocityRmsAxis(Axis)` — RMS prędkości wzdłuż jednej osi.
- `AttitudeRms` — RMS błędu orientacji (roll² + pitch²) [rad].
- `AttitudeMaxError` — maksymalny błąd orientacji √(roll² + pitch²) [rad].
- `OvershootPercent` — przeregulowanie wyrażone w procentach zakresu [%].
- `SettlingTimeS` — czas ustalania (próg 0.1 m) [s].
- `RiseTimeS` — czas narastania (10%–90% zakresu) [s].
- `SteadyStateError` — błąd stanu ustalonego (średni błąd z ostatnich 20% symulacji) [m].
- `ControlEnergy` — zużycie energii sterowania (przybliżenie ω³) [j.u.].
- `MaxControlRate` — maksymalna szybkość zmiany sygnału sterującego [rad/s²].

Pomocniczy enum `Axis`:

```rust path=null start=null
pub enum Axis { X, Y, Z }
```

W TOML metryki z osią zapisuje się np.: `metric = { position_rms_axis = "z" }`.

### `ScenarioTrajectoryDef`

Definicja trajektorii czasowej. Pole `type` w TOML wybiera wariant.

```rust path=null start=null
pub enum ScenarioTrajectoryDef {
    Hold { z: f64, x: Option<f64>, y: Option<f64>, yaw: Option<f64> },
    Waypoint { waypoints: Vec<WaypointEntry> },
    Circle { cx: f64, cy: f64, radius: f64, omega_deg_s: f64, altitude_m: f64 },
}
```

Warianty:

- `Hold` — utrzymanie stałej pozycji. Pola: `z` (wymagane), `x`, `y`, `yaw` (opcjonalne).
- `Waypoint` — ścieżka liniowa przez punkty drogi z czasami. Pole `waypoints` — lista `WaypointEntry`.
- `Circle` — orbita kołowa w poziomie. Pola:
  - `cx`, `cy` — środek okręgu [m].
  - `radius` — promień [m].
  - `omega_deg_s` — prędkość kątowa [°/s] (konwertowana na rad/s wewnętrznie).
  - `altitude_m` — wysokość orbity [m].

Metoda `into_trajectory()` konwertuje definicję do obiektu `Box<dyn Trajectory>`.

### `WaypointEntry`

Pojedynczy punkt drogi w trajektorii `Waypoint`.

```rust path=null start=null
pub struct WaypointEntry {
    pub time_s: f64,
    pub z: f64,
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub yaw: Option<f64>,
}
```

Pola:

- `time_s` — czas, w którym punkt powinien być osiągnięty [s].
- `z` — wysokość [m] — **wymagane**.
- `x` — pozycja X [m] — opcjonalnie.
- `y` — pozycja Y [m] — opcjonalnie.
- `yaw` — kąt odchylenia [rad] — opcjonalnie.

### Kompletny przykład TOML

```toml path=null start=null
name = "hover_z5_with_gust"
description = "Wznoszenie na 5m z porywem wiatru w 3. sekundzie"
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

Przykład z trajektorią kołową:

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

## 3. Moduł `controller_config`

Moduł zawiera typy konfiguracyjne regulatorów lotu oraz fabryki tworzące instancje regulatorów.

### `ControllerConfig`

Główny enum wybierający i konfigurujący regulator. Deserializowany z TOML z tagiem wewnętrznym `type`.

```rust path=null start=null
pub enum ControllerConfig {
    Cascade(CascadeConfig),
    Lqr(LqrConfig),
    Lqi(LqiConfig),
}
```

Warianty:

- `Cascade` — kaskadowy regulator PID. TOML: `type = "cascade"`.
- `Lqr` — regulator liniowo-kwadratowy (Linear-Quadratic Regulator). TOML: `type = "lqr"`.
- `Lqi` — regulator liniowo-kwadratowy z całkowaniem (LQR + stany integralne). TOML: `type = "lqi"`.

Domyślna implementacja (`Default`) zwraca `Cascade(CascadeConfig::default())`.

#### `name()`

```rust path=null start=null
pub fn name(&self) -> &str
```

Zwraca czytelną nazwę regulatora: `"Cascade-PID"`, `"LQR"` lub `"LQI"`.

#### `from_file()`

```rust path=null start=null
pub fn from_file(path: &Path) -> anyhow::Result<Self>
```

Wczytuje konfigurację regulatora z pliku TOML.

#### `into_factory()`

```rust path=null start=null
pub fn into_factory(self) -> ControllerFactory
```

Konwertuje konfigurację w domknięcie (`ControllerFactory`), które tworzy świeżą instancję regulatora. Fabryka przyjmuje referencję do modelu pojazdu (potrzebną do wyznaczenia punktu równowagi / linearyzacji) i zwraca `Box<dyn Controller>`.

### `PidConfig`

Parametry pojedynczej pętli PID.

```rust path=null start=null
pub struct PidConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limit: f64,
    pub output_limit: f64,
}
```

Pola:

- `kp` — wzmocnienie proporcjonalne.
- `ki` — wzmocnienie całkujące.
- `kd` — wzmocnienie różniczkujące.
- `integral_limit` — ograniczenie anti-windup na akumulatorze całki.
- `output_limit` — ograniczenie na całkowitym wyjściu pętli.

### `CascadeConfig`

Konfiguracja kaskadowego regulatora PID. Kaskada ma trzy poziomy: pozycja → prędkość (zewnętrzny), prędkość → orientacja (środkowy), orientacja → komendy silników (wewnętrzny).

```rust path=null start=null
pub struct CascadeConfig {
    pub max_tilt_deg: f64,
    pub vel_z: PidConfig,
    pub vel_xy: PidConfig,
    pub att: PidConfig,
    pub att_yaw: PidConfig,
}
```

Pola:

- `max_tilt_deg` — maksymalne przechylenie poziome [°]. Domyślnie 8.6° — zapobiega nasyceniu silników przy jednoczesnym roll i pitch.
- `vel_z` — pętla prędkości pionowej: błąd vz → delta przepustnicy.
- `vel_xy` — pętla prędkości poziomej: błąd vx/vy → docelowy pitch/roll. Ta sama konfiguracja dla obu osi X i Y.
- `att` — pętla orientacji: błąd roll/pitch → delta komendy silnika. Ta sama konfiguracja dla roll i pitch.
- `att_yaw` — pętla orientacji yaw: błąd yaw → delta komendy silnika.

Wartości domyślne (`Default`):

- `max_tilt_deg = 8.6`
- `vel_z`: kp=0.3, ki=0.1, kd=0.0, integral_limit=0.45, output_limit=0.45
- `vel_xy`: kp=0.4, ki=0.05, kd=0.0, integral_limit=0.5, output_limit=0.35
- `att`: kp=4.0, ki=0.0, kd=0.2, integral_limit=1.0, output_limit=1.0
- `att_yaw`: kp=2.0, ki=0.1, kd=0.0, integral_limit=0.5, output_limit=0.5

### `LqrConfig`

Konfiguracja regulatora LQR (Linear-Quadratic Regulator). Równanie CARE jest rozwiązywane jednorazowo wokół ustalonego punktu równowagi. LQR stabilizuje punkt trymowania — **nie** śledzi dowolnych setpointów (do śledzenia użyj `LqiConfig`).

Obsługiwane wyłącznie pojazdy typu quadrotor.

```rust path=null start=null
pub struct LqrConfig {
    pub trim_z_m: f64,
    pub q_weights: Option<Vec<f64>>,
    pub r_weights: Option<Vec<f64>>,
}
```

Pola:

- `trim_z_m` — wysokość trymowania do linearyzacji [m]. Domyślnie 5.0.
- `q_weights` — wektor wag Q — 13 elementów dla quadrotora: `[x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]`. Opcjonalnie — domyślne wagi wbudowane: `[3.0, 3.0, 80.0, 0.5, 0.5, 10.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0]`.
- `r_weights` — wektor wag R — 4 elementy (po jednym na silnik). Większe wartości → gładniejsze, mniej agresywne sterowanie. Domyślnie `[0.01, 0.01, 0.01, 0.01]`.

### `LqiConfig`

Konfiguracja regulatora LQI (Linear-Quadratic Integral). Rozszerza LQR o cztery stany integralne `[ξ_x, ξ_y, ξ_z, ξ_ψ]`, eliminujące błąd stanu ustalonego przy stałych zakłóceniach.

Obsługiwane wyłącznie pojazdy typu quadrotor.

```rust path=null start=null
pub struct LqiConfig {
    pub trim_z_m: f64,
    pub q_weights: Option<Vec<f64>>,
    pub r_weights: Option<Vec<f64>>,
    pub xi_limits: Option<[f64; 4]>,
}
```

Pola:

- `trim_z_m` — wysokość trymowania do linearyzacji [m]. Domyślnie 5.0.
- `q_weights` — wektor wag Q — 17 elementów: 13 stanów procesu + 4 stany integralne `[ξ_x, ξ_y, ξ_z, ξ_ψ]`. Domyślne: `[1.0, 1.0, 100.0, 0.5, 0.5, 12.0, 2.0, 2.0, 2.0, 20.0, 20.0, 20.0, 20.0, 5.0, 5.0, 6.0, 2.0]`.
- `r_weights` — wektor wag R — 4 elementy. Domyślnie `[0.005, 0.005, 0.005, 0.005]`.
- `xi_limits` — ograniczenia anti-windup `[m·s, m·s, m·s, rad·s]` na czterech stanach integralnych. Domyślnie `[30, 30, 30, 2π]`.

---

## 4. Moduł `runner`

Pętla symulacji SITL. Łączy scenariusz, model pojazdu, regulator i zakłócenia w jedną symulację.

### `ControllerFactory`

Alias typu — fabryka tworząca świeży regulator dla danego modelu pojazdu.

```rust path=null start=null
pub type ControllerFactory =
    Box<dyn Fn(&dyn VehicleModel) -> Result<Box<dyn Controller>> + Send + Sync>;
```

Fabryka (zamiast gotowej instancji) gwarantuje, że każdy przebieg symulacji startuje z czystym stanem regulatora.

### `run_scenario()`

```rust path=null start=null
pub fn run_scenario(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
) -> Result<ScenarioReport>
```

Uruchamia scenariusz SITL. Jeśli scenariusz definiuje trajektorię, jest ona używana automatycznie (nadpisuje statyczny `[target]`). W przeciwnym wypadku używany jest statyczny cel.

Parametry:

- `scenario` — definicja scenariusza (czas, cel, zakłócenia, asercje).
- `model` — model pojazdu (np. `QuadrotorModel::mini3()`).
- `factory` — fabryka regulatora.

Zwraca `ScenarioReport` z wynikami asercji i historią klatek.

### `run_scenario_with_trajectory()`

```rust path=null start=null
pub fn run_scenario_with_trajectory(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
    trajectory: &dyn Trajectory,
) -> Result<ScenarioReport>
```

Uruchamia scenariusz z trajektorią czasową zamiast statycznego celu. W każdym kroku symulacji wywoływane jest `trajectory.target(time_s)`. Asercje ewaluowane są względem końcowego celu trajektorii.

### Funkcje wewnętrzne (`pub(crate)`)

#### `run_with_disturbances()`

Kanoniczna pętla symulacji. W każdym kroku:
1. Aplikuje aktywne zakłócenia (`disturbance.apply()`).
2. Wywołuje regulator (`controller.update()`).
3. Aktualizuje dynamikę aktuatorów (`model.step_actuators()`).
4. Całkuje stan metodą RK4.

Zwraca pełną historię klatek `Vec<SimFrame>`.

#### `run_with_disturbances_traj()`

Jak `run_with_disturbances`, ale zamiast stałego celu wywołuje `trajectory.target(time)` w każdym kroku.

#### `scenario_to_flight_target()`

Konwertuje `ScenarioTarget` na `FlightTarget` z biblioteki sterowania. Pole `z` (wymagane w TOML) jest zawsze `Some`; pola `x`, `y`, `yaw` pozostają `None` jeśli nieobecne — oznacza to brak kontroli danej osi.

---

## 5. Moduł `disturbance`

Moduł modelujący zakłócenia zewnętrzne działające na dron podczas symulacji.

### Trait `Disturbance`

```rust path=null start=null
pub trait Disturbance: Send + Sync {
    fn is_active(&self, time: f64) -> bool;
    fn apply(&self, state: &mut DroneState, model: &dyn VehicleModel, dt: TimeStep);
}
```

Metody:

- `is_active(time)` — sprawdza, czy zakłócenie jest aktywne w danej chwili `time` [s].
- `apply(state, model, dt)` — modyfikuje stan drona (`DroneState`) zgodnie z modelem zakłócenia. Otrzymuje model pojazdu (np. do pobrania masy) i krok czasowy.

### `DisturbanceConfig`

Enum konfiguracyjny deserializowany z TOML. Tag wewnętrzny `type` wybiera wariant.

```rust path=null start=null
pub enum DisturbanceConfig {
    WindGust(WindGustConfig),
    Turbulence(TurbulenceConfig),
    MotorFailure(MotorFailureConfig),
}
```

Metoda `into_disturbance()` tworzy odpowiednią instancję `Box<dyn Disturbance>`.

### `WindGust` / `WindGustConfig`

Impulsowa siła wiatru działająca w określonym przedziale czasowym.

```rust path=null start=null
pub struct WindGustConfig {
    pub at_s: f64,
    pub duration_s: f64,  // domyślnie 0.1
    pub force: [f64; 3],
}
```

Pola:

- `at_s` — czas rozpoczęcia porywu [s].
- `duration_s` — czas trwania porywu [s] (domyślnie 0.1 s).
- `force` — wektor siły `[Fx, Fy, Fz]` [N].

Aktywny w przedziale `[at_s, at_s + duration_s)`. Działanie: dodaje zmianę prędkości `Δv = F · dt / m` do stanu drona (impuls siły).

TOML:
```toml path=null start=null
[[disturbances]]
type = "wind_gust"
at_s = 3.0
duration_s = 0.2
force = [2.0, 0.0, -1.0]
```

### `Turbulence` / `TurbulenceConfig`

Ciągły szum Gaussa modelujący turbulencje atmosferyczne.

```rust path=null start=null
pub struct TurbulenceConfig {
    pub start_s: f64,
    pub end_s: f64,
    pub intensity_n: f64,
    pub seed: u64,
    pub z_only: bool,
}
```

Pola:

- `start_s` — czas rozpoczęcia turbulencji [s].
- `end_s` — czas zakończenia turbulencji [s].
- `intensity_n` — intensywność (odchylenie standardowe rozkładu normalnego siły) [N].
- `seed` — ziarno generatora liczb losowych (domyślnie 0). Zapewnia deterministyczną powtarzalność.
- `z_only` — jeśli `true`, turbulencje działają tylko na oś Z; osie X/Y pozostają niezakłócone. Przydatne do testowania tłumienia zakłóceń wysokościowych bez sprzężeń XY→Z.

Aktywna w przedziale `[start_s, end_s)`. Każdy krok symulacji losuje siłę z rozkładu N(0, intensity_n²) i dodaje `Δv = F · dt / m`. Generator: `SmallRng` (szybki, deterministyczny).

### `MotorFailure` / `MotorFailureConfig`

Permanentna awaria jednego silnika. Symuluje niezrównoważony moment obrotowy.

```rust path=null start=null
pub struct MotorFailureConfig {
    pub at_s: f64,
    pub motor_index: usize,
}
```

Pola:

- `at_s` — czas awarii [s].
- `motor_index` — indeks uszkodzonego silnika (0–3 dla quadrotora).

Aktywna od `at_s` do końca symulacji (permanentna). Dodaje moment obrotowy yaw proporcjonalny do kwadratu prędkości silnika w stanie równowagi (`k_torque ≈ 1.5e-8`). Znak momentu zależy od parzystości indeksu silnika.

---

## 6. Moduł `metrics`

Funkcje obliczające metryki jakości sterowania na podstawie historii klatek symulacji.

### `compute()`

```rust path=null start=null
pub fn compute(metric: &MetricKind, frames: &[SimFrame], target: &FlightTarget) -> f64
```

Dispatcher — wywołuje odpowiednią funkcję metryki na podstawie wariantu `MetricKind`. Zwraca wartość metryki jako `f64`.

### Metryki pozycji

#### `position_rms_3d()`

```rust path=null start=null
pub fn position_rms_3d(frames: &[SimFrame], target: &FlightTarget) -> f64
```

RMS błędu pozycji 3D.

Wzór: `√( (1/N) · Σ ||p_i - p_target||² )`

#### `position_rms_axis()`

```rust path=null start=null
pub fn position_rms_axis(frames: &[SimFrame], target: &FlightTarget, axis: &Axis) -> f64
```

RMS błędu pozycji wzdłuż jednej osi (X, Y lub Z).

Wzór: `√( (1/N) · Σ (p_i[oś] - p_target[oś])² )`

#### `position_rms_z()`

```rust path=null start=null
pub fn position_rms_z(frames: &[SimFrame], target: &FlightTarget) -> f64
```

RMS błędu pozycji Z (wysokość). Wersja legacy używana w `comparison.rs`.

Wzór: `√( (1/N) · Σ (z_i - z_target)² )`

#### `position_max_error_3d()`

```rust path=null start=null
pub fn position_max_error_3d(frames: &[SimFrame], target: &FlightTarget) -> f64
```

Maksymalny błąd pozycji 3D w całym locie.

Wzór: `max( ||p_i - p_target|| )`

#### `position_max_error_axis()`

```rust path=null start=null
pub fn position_max_error_axis(frames: &[SimFrame], target: &FlightTarget, axis: &Axis) -> f64
```

Maksymalny błąd pozycji wzdłuż jednej osi.

Wzór: `max( |p_i[oś] - p_target[oś]| )`

#### `position_max_error_z()`

```rust path=null start=null
pub fn position_max_error_z(frames: &[SimFrame], target: &FlightTarget) -> f64
```

Maksymalny błąd pozycji Z. Wersja legacy.

### Metryki prędkości

Referencja = 0 (odchylenie od spoczynku).

#### `velocity_rms_3d()`

```rust path=null start=null
pub fn velocity_rms_3d(frames: &[SimFrame]) -> f64
```

RMS prędkości 3D.

Wzór: `√( (1/N) · Σ ||v_i||² )`

#### `velocity_rms_axis()`

```rust path=null start=null
pub fn velocity_rms_axis(frames: &[SimFrame], axis: &Axis) -> f64
```

RMS prędkości wzdłuż jednej osi.

### Metryki orientacji

Referencja = lot poziomy (roll = 0, pitch = 0).

#### `attitude_rms()`

```rust path=null start=null
pub fn attitude_rms(frames: &[SimFrame]) -> f64
```

RMS błędu orientacji.

Wzór: `√( (1/N) · Σ (roll² + pitch²) )` [rad]

#### `attitude_max_error()`

```rust path=null start=null
pub fn attitude_max_error(frames: &[SimFrame]) -> f64
```

Maksymalny błąd orientacji.

Wzór: `max( √(roll² + pitch²) )` [rad]

### Metryki odpowiedzi skokowej

#### `overshoot_percent()`

```rust path=null start=null
pub fn overshoot_percent(frames: &[SimFrame], target: &FlightTarget) -> f64
```

Przeregulowanie — o ile procent dron przekroczył cel względem całkowitego zakresu.

Wzór: `((z_max - z_target) / (z_target - z_initial)) · 100%`

Zwraca 0.0 jeśli dron nigdy nie osiągnął ani nie przekroczył celu.

#### `settling_time_s()`

```rust path=null start=null
pub fn settling_time_s(frames: &[SimFrame], target: &FlightTarget) -> f64
```

Czas ustalania — czas [s] ostatniego momentu, gdy błąd Z ≥ 0.1 m (próg stały = 0.1 m). Po tym czasie dron pozostaje w pasmie tolerancji.

#### `rise_time_s()`

```rust path=null start=null
pub fn rise_time_s(frames: &[SimFrame], target_z: f64) -> f64
```

Czas narastania — czas przejścia od 10% do 90% całkowitej zmiany pozycji Z.

Wzór: `t_90% - t_10%`

Zwraca `f64::INFINITY` jeśli 90% nie zostało osiągnięte.

#### `steady_state_error()`

```rust path=null start=null
pub fn steady_state_error(frames: &[SimFrame], target_z: f64) -> f64
```

Błąd stanu ustalonego — średni bezwzględny błąd Z w ostatnich 20% czasu symulacji.

Wzór: `(1/N_late) · Σ |z_i - z_target|` dla klatek z `t ≥ 0.8 · t_max`

### Metryki energetyczne

#### `control_energy()`

```rust path=null start=null
pub fn control_energy(frames: &[SimFrame]) -> f64
```

Przybliżenie całkowitej energii zużytej przez silniki.

W modelu śmigła moment obrotowy τ = k_torque · ω², zatem moc mechaniczna P = τ · ω = k_torque · **ω³**. Całka Σ ω³ · dt po wszystkich silnikach daje wielkość proporcjonalną do energii (k_torque upraszcza się przy porównywaniu regulatorów na tym samym pojeździe).

Użycie ω³ (zamiast naiwnego ω²) zapewnia poprawną kolejność rankingową dla profili łączących krótkie wybuchy wysokiego RPM z umiarkowanym ciągłym RPM.

#### `max_control_rate()`

```rust path=null start=null
pub fn max_control_rate(frames: &[SimFrame]) -> f64
```

Maksymalna szybkość zmiany sygnału sterującego — max z `|ω[i](t+1) - ω[i](t)| / dt` po wszystkich silnikach i krokach czasowych [rad/s²].

---

## 7. Moduł `report`

Struktury raportujące wyniki pojedynczego scenariusza.

### `AssertionResult`

Wynik pojedynczej asercji.

```rust path=null start=null
pub struct AssertionResult {
    pub metric: String,
    pub value: f64,
    pub max: f64,
    pub passed: bool,
}
```

Pola:

- `metric` — nazwa metryki (np. `"OvershootPercent"`).
- `value` — obliczona wartość metryki.
- `max` — próg z asercji scenariusza.
- `passed` — `true` jeśli `value ≤ max`.

### `ScenarioReport`

Pełny raport ze scenariusza.

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

Pola:

- `name` — nazwa scenariusza.
- `passed` — `true` jeśli wszystkie asercje spełnione.
- `duration_s` — czas trwania symulacji [s].
- `frame_count` — liczba klatek w historii.
- `assertions` — wyniki poszczególnych asercji.
- `frames` — pełna historia klatek symulacji.

#### `print()`

```rust path=null start=null
pub fn print(&self)
```

Wyświetla raport na stdout w formacie czytelnym dla człowieka (status PASS/FAIL, czasy, wyniki asercji z symbolami ✓/✗).

#### `to_csv()`

```rust path=null start=null
pub fn to_csv(&self) -> String
```

Serializuje historię klatek do formatu CSV. Kolumny: `time,x,y,z,vx,vy,vz`.

---

## 8. Moduł `comparison`

Porównywanie wielu regulatorów na tym samym scenariuszu.

### `ControllerResult`

Wynik jednego regulatora w porównaniu.

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

Pola:

- `name` — nazwa regulatora.
- `frames` — historia klatek symulacji.
- `rms_error_z` — RMS błędu Z [m].
- `max_error_z` — maksymalny błąd Z [m].
- `overshoot_pct` — przeregulowanie [%].
- `settling_time_s` — czas ustalania [s].
- `rise_time_s` — czas narastania [s].
- `steady_state_err` — błąd stanu ustalonego [m].
- `control_energy` — energia sterowania [j.u.].
- `max_control_rate` — maks. szybkość zmiany sterowania [rad/s²].

### `ComparisonReport`

Raport zbiorczy porównania regulatorów.

```rust path=null start=null
pub struct ComparisonReport {
    pub scenario_name: String,
    pub target_z: f64,
    pub results: Vec<ControllerResult>,
}
```

Pola:

- `scenario_name` — nazwa scenariusza.
- `target_z` — docelowa wysokość [m].
- `results` — wyniki poszczególnych regulatorów.

#### `print()`

```rust path=null start=null
pub fn print(&self)
```

Wyświetla tabelę porównawczą z ramką Unicode. Zawiera metryki (RMS, OS, ST, RT, Energia) oraz podsumowanie najlepszych regulatorów (Best RMS, Best energy, Quickest).

#### `to_csv()`

```rust path=null start=null
pub fn to_csv(&self) -> String
```

Eksportuje metryki do CSV. Kolumny: `controller,rms_z,max_error_z,overshoot_pct,settling_time_s,rise_time_s,steady_state_err,control_energy,max_control_rate`.

#### `trajectories_to_csv()`

```rust path=null start=null
pub fn trajectories_to_csv(&self) -> String
```

Eksportuje trajektorie wszystkich regulatorów do jednego CSV. Kolumny: `time,z_{nazwa1},vz_{nazwa1},z_{nazwa2},vz_{nazwa2},...`.

### `compare_controllers()`

```rust path=null start=null
pub fn compare_controllers(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factories: &[(&str, ControllerFactory)],
) -> Result<ComparisonReport>
```

Porównuje wiele regulatorów na tym samym scenariuszu. Dla każdego regulatora:
1. Tworzy instancję z fabryki.
2. Uruchamia symulację z zakłóceniami scenariusza.
3. Oblicza pełen zestaw metryk.

Parametry:

- `scenario` — definicja scenariusza.
- `model` — model pojazdu.
- `factories` — lista par `(nazwa, fabryka)`.

Zwraca `ComparisonReport` ze wszystkimi wynikami.

---

## 9. Moduł `monte_carlo`

Analiza Monte Carlo — uruchamianie scenariusza wielokrotnie z zaburzeniami warunków początkowych i agregacja statystyk metryk.

### `MonteCarloConfig`

Konfiguracja przeglądu Monte Carlo.

```rust path=null start=null
pub struct MonteCarloConfig {
    pub runs: usize,
    pub pos_noise_m: f64,
    pub vel_noise_ms: f64,
    pub base_seed: u64,
}
```

Pola:

- `runs` — liczba niezależnych przebiegów symulacji. Domyślnie 100.
- `pos_noise_m` — odchylenie standardowe szumu gaussowskiego pozycji [m]. Domyślnie 0.5.
- `vel_noise_ms` — odchylenie standardowe szumu gaussowskiego prędkości [m/s]. Domyślnie 0.1.
- `base_seed` — bazowe ziarno generatora; przebieg `i` używa ziarna `base_seed + i` dla reprodukowalności. Domyślnie 42.

Perturbacje: X, Y — normalne; Z — `|perturbacja|` (zawsze ≥ 0, aby zapobiec negatywnej wysokości); vx, vy, vz — normalne.

### `MetricStats`

Statystyki jednej metryki zagregowane po wszystkich przebiegach Monte Carlo.

```rust path=null start=null
pub struct MetricStats {
    pub name: String,
    pub threshold: f64,
    pub mean: f64,
    pub std_dev: f64,
    pub min: f64,
    pub max: f64,
    pub pass_rate: f64,
}
```

Pola:

- `name` — nazwa metryki (np. `"OvershootPercent"`).
- `threshold` — próg asercji ze scenariusza.
- `mean` — średnia wartość metryki.
- `std_dev` — odchylenie standardowe.
- `min` — minimalna zaobserwowana wartość.
- `max` — maksymalna zaobserwowana wartość.
- `pass_rate` — frakcja przebiegów, w których metryka ≤ threshold (0.0–1.0).

### `MonteCarloReport`

Pełny raport z przeglądu Monte Carlo.

```rust path=null start=null
pub struct MonteCarloReport {
    pub scenario_name: String,
    pub runs: usize,
    pub metrics: Vec<MetricStats>,
}
```

Pola:

- `scenario_name` — nazwa scenariusza.
- `runs` — liczba wykonanych przebiegów.
- `metrics` — zagregowane statystyki poszczególnych metryk.

#### `print()`

```rust path=null start=null
pub fn print(&self)
```

Wyświetla tabelę z ramką Unicode: Mean, StdDev, Min, Max, Thresh, Pass%.

#### `to_csv()`

```rust path=null start=null
pub fn to_csv(&self) -> String
```

Serializuje do CSV. Kolumny: `metric,threshold,mean,std_dev,min,max,pass_rate`.

### `run_monte_carlo()`

```rust path=null start=null
pub fn run_monte_carlo(
    scenario: &Scenario,
    model: &(dyn VehicleModel + Send + Sync),
    factory: &ControllerFactory,
    cfg: &MonteCarloConfig,
) -> MonteCarloReport
```

Uruchamia scenariusz `cfg.runs` razy **równolegle** za pomocą Rayon (`into_par_iter()`).

Każdy przebieg niezależnie:
1. Tworzy generator `SmallRng` z ziarnem `base_seed + i`.
2. Perturbuje warunki początkowe szumem gaussowskim.
3. Uruchamia `run_scenario()`.
4. Zbiera wyniki asercji.

Jeśli symulacja zakończy się błędem, wartości metryk są ustawiane na `f64::INFINITY` (przebieg traktowany jako niezdany).

Po zakończeniu wszystkich przebiegów obliczane są statystyki (mean, std_dev, min, max, pass_rate) dla każdej metryki.

Parametry:

- `scenario` — definicja scenariusza.
- `model` — model pojazdu (musi implementować `Send + Sync`).
- `factory` — fabryka regulatora (`Send + Sync` umożliwia wywołanie z wielu wątków Rayon).
- `cfg` — konfiguracja Monte Carlo.
