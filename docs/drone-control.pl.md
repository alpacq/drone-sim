# drone-control — dokumentacja referencyjna

## 1. Przegląd

Crate `drone-control` dostarcza kompletny stos regulatorów lotu dla symulatora dronów. Zawiera:

- **Trait `Controller`** — ujednolicony interfejs regulatora lotu.
- **`FlightTarget`** — opis zadanego stanu lotu z opcjonalnymi osiami.
- **Regulator PID** z zabezpieczeniem anti-windup.
- **Pętlę wewnętrzną (inner loop)** — konwersja błędu prędkości na sygnał sterujący.
- **Profile prędkości (velocity profiler)** — pętla zewnętrzna: pozycja → prędkość zadana.
- **Mikser (mixer)** — przeliczanie komend kątowych na sygnały aktuatorów.
- **Regulator kaskadowy** — trzypoziomowa kaskada: pozycja → prędkość → kąty → silniki.
- **LQR / LQI** — regulator liniowo-kwadratowy z opcjonalnym członem całkującym do eliminacji uchybu ustalonego.
- **Trajektorie** — generatory zmiennych w czasie celów lotu.

---

## 2. Moduł `controller`

Definiuje trait wspólny dla wszystkich regulatorów lotu.

### Trait `Controller`

```rust
pub trait Controller: Send + Sync {
    fn update(&mut self, state: &DroneState, target: &FlightTarget, dt: TimeStep) -> KnownActuatorInput;
    fn reset(&mut self);
    fn name(&self) -> &str;
}
```

#### Metody

- **`update(&mut self, state: &DroneState, target: &FlightTarget, dt: TimeStep) -> KnownActuatorInput`**
  Oblicza wyjście regulatora (sygnały aktuatorów) na podstawie aktualnego stanu drona `state`, celu lotu `target` i kroku czasowego `dt`. Wywoływana co krok symulacji.

- **`reset(&mut self)`**
  Zeruje stan wewnętrzny regulatora (całki, poprzednie błędy). Używana przy zmianie trybu lotu lub restarcie kontrolera.

- **`name(&self) -> &str`**
  Zwraca nazwę regulatora (np. `"CascadeController"`, `"LQR"`, `"LqiController"`). Służy do logowania i diagnostyki.

---

## 3. Moduł `target`

Opisuje zadany stan lotu jako zbiór opcjonalnych wartości zadanych na poszczególnych osiach.

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

#### Pola

- **`x: Option<f64>`** — zadana pozycja X [m]. `None` = oś X nie jest sterowana.
- **`y: Option<f64>`** — zadana pozycja Y [m]. `None` = oś Y nie jest sterowana.
- **`z: Option<f64>`** — zadana wysokość Z [m]. `None` = wysokość nie jest sterowana.
- **`yaw: Option<f64>`** — zadany kąt odchylenia (yaw) [rad]. `None` = yaw nie jest sterowany.

Semantyka `None`: regulator kaskadowy i integratory LQI traktują brakującą oś jako zerowy błąd i zerową akumulację całki — dron stabilizuje się w bieżącej pozycji na tej osi, zamiast dążyć do zera.

#### Metody fabryczne

- **`FlightTarget::altitude(z: f64) -> Self`**
  Cel tylko wysokościowy. Tylko Z jest sterowane; X, Y i yaw ustawione na `None`.

- **`FlightTarget::position(x: f64, y: f64, z: f64) -> Self`**
  Cel 3D pozycyjny (bez sterowania yaw). X, Y, Z ustawione na `Some`; yaw = `None`.

- **`FlightTarget::full(x: f64, y: f64, z: f64, yaw: f64) -> Self`**
  Pełny cel 3D + yaw. Wszystkie cztery osie ustawione na `Some`.

---

## 4. Moduł `pid`

Implementacja regulatora PID z zabezpieczeniem anti-windup.

### Struct `Pid`

```rust
#[derive(Debug, Clone)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limit: f64,
    pub output_limit: f64,
    // pola prywatne: integral, prev_error
}
```

#### Pola publiczne

- **`kp: f64`** — wzmocnienie proporcjonalne.
- **`ki: f64`** — wzmocnienie całkujące.
- **`kd: f64`** — wzmocnienie różniczkujące.
- **`integral_limit: f64`** — maksymalna wartość bezwzględna członu całkującego (ochrona anti-windup).
- **`output_limit: f64`** — maksymalna wartość bezwzględna wyjścia regulatora.

#### Metody

- **`Pid::new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self`**
  Tworzy nowy regulator PID z podanymi parametrami. Stan wewnętrzny inicjalizowany zerami.

- **`update(&mut self, error: f64, dt: TimeStep) -> f64`**
  Oblicza wyjście regulatora dla danego błędu i kroku czasowego:
  - P = `kp × error`
  - I = `ki × ∫error·dt` (z clampowaniem do `±integral_limit`)
  - D = `kd × (error − prev_error) / dt`
  - Wyjście = `clamp(P + I + D, ±output_limit)`

- **`reset(&mut self)`**
  Zeruje stan wewnętrzny (`integral = 0`, `prev_error = 0`).

---

## 5. Moduł `inner_loop`

Pętla wewnętrzna regulatora kaskadowego: błąd prędkości → sygnał sterujący. Posiada stan wewnętrzny (pamięć).

### Trait `InnerLoop`

```rust
pub trait InnerLoop: Send + Sync {
    fn compute(&mut self, error: f64, dt: TimeStep) -> f64;
    fn reset(&mut self);
}
```

#### Metody

- **`compute(&mut self, error: f64, dt: TimeStep) -> f64`**
  Oblicza wyjście sterujące dla danego błędu i kroku czasowego.

- **`reset(&mut self)`**
  Zeruje stan wewnętrzny pętli.

### Struct `PidLoop`

```rust
pub struct PidLoop(pub Pid);
```

Opakowuje `Pid` i implementuje trait `InnerLoop`. Deleguje `compute` do `Pid::update` i `reset` do `Pid::reset`.

#### Metody

- **`PidLoop::new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self`**
  Tworzy nowy `PidLoop` z regulatorem PID o podanych parametrach.

---

## 6. Moduł `profiler`

Pętla zewnętrzna regulatora kaskadowego: pozycja → prędkość zadana. Profilery są bezstanowe — to samo wejście zawsze daje to samo wyjście.

### Trait `VelocityProfiler`

```rust
pub trait VelocityProfiler: Send + Sync {
    fn compute(&self, error: f64) -> f64;
}
```

#### Metody

- **`compute(&self, error: f64) -> f64`**
  Oblicza zadaną prędkość [m/s] dla danego błędu pozycji [m].

### Struct `SqrtProfiler`

Profiler pierwiastkowy — kinematyczny profil hamowania.

```rust
pub struct SqrtProfiler {
    pub brake_accel: f64,
    pub v_max: f64,
}
```

#### Pola

- **`brake_accel: f64`** — przyspieszenie hamowania [m/s²].
- **`v_max: f64`** — maksymalna prędkość zbliżania [m/s].

#### Formuła

```
v = sign(e) × min(√(2 · brake_accel · |e|), v_max)
```

Przy małych błędach prędkość rośnie łagodnie (proporcjonalnie do √|e|), przy dużych jest ograniczona do `v_max`. Zapewnia płynne hamowanie bez przekroczenia celu.

#### Metody

- **`SqrtProfiler::new(brake_accel: f64, v_max: f64) -> Self`**
  Tworzy profiler z podanymi parametrami.

- **`SqrtProfiler::for_altitude() -> Self`**
  Predefiniowany profiler dla osi Z: `brake_accel = 1.5 m/s²`, `v_max = 1.0 m/s`.

- **`SqrtProfiler::for_horizontal() -> Self`**
  Predefiniowany profiler dla płaszczyzny XY: `brake_accel = 2.0 m/s²`, `v_max = 3.0 m/s`.

### Struct `LinearProfiler`

Profiler liniowy — prosta zależność proporcjonalna.

```rust
pub struct LinearProfiler {
    pub kp: f64,
    pub v_max: f64,
}
```

#### Pola

- **`kp: f64`** — wzmocnienie proporcjonalne.
- **`v_max: f64`** — maksymalna prędkość [m/s].

#### Formuła

```
v = clamp(kp × error, -v_max, v_max)
```

#### Metody

- **`LinearProfiler::new(kp: f64, v_max: f64) -> Self`**
  Tworzy profiler liniowy z podanymi parametrami.

---

## 7. Moduł `mixer`

Przelicza wysokopoziomowe komendy orientacji na konkretne sygnały aktuatorów.

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

#### Pola

- **`throttle: f64`** — przepustnica [0, 1].
- **`roll: f64`** — komenda przechylenia [-1, 1].
- **`pitch: f64`** — komenda pochylenia [-1, 1].
- **`yaw: f64`** — komenda odchylenia [-1, 1].

### Trait `Mixer`

```rust
pub trait Mixer: Send + Sync {
    fn mix(&self, cmd: &AttitudeCommand) -> KnownActuatorInput;
    fn equilibrium_command(&self) -> AttitudeCommand;
}
```

#### Metody

- **`mix(&self, cmd: &AttitudeCommand) -> KnownActuatorInput`**
  Przelicza komendę orientacji na sygnały aktuatorów (prędkości silników dla quadrotora lub wychylenia powierzchni sterowych dla samolotu).

- **`equilibrium_command(&self) -> AttitudeCommand`**
  Zwraca komendę odpowiadającą stanowi równowagi (zawis/lot ustalony). Używana jako punkt pracy w regulatorze kaskadowym.

### Struct `QuadrotorMixer`

Mikser dla quadrotora w konfiguracji ramienia X.

```rust
pub struct QuadrotorMixer {
    hover_motor_speed: f64,
    max_motor_speed: f64,
}
```

#### Geometria X-frame (widok z góry)

```
  1(CCW)  0(CW)
     \   /
      [B]     ← przód (+x)
     /   \
  2(CW)  3(CCW)
```

- Silnik 0 (FrontRight, CW): `base - p - r + y`
- Silnik 1 (FrontLeft, CCW): `base - p + r - y`
- Silnik 2 (RearLeft, CW): `base + p + r + y`
- Silnik 3 (RearRight, CCW): `base + p - r - y`

Gdzie `base = throttle × max_motor_speed`, `r/p/y = roll/pitch/yaw × max_motor_speed × 0.5`.

#### Metody

- **`QuadrotorMixer::new(hover_motor_speed: f64, max_motor_speed: f64) -> Self`**
  Tworzy mikser z podanymi prędkościami silników.

- **`QuadrotorMixer::from_equilibrium(input: KnownActuatorInput) -> Self`**
  Tworzy mikser na podstawie wejścia równowagi (średnia prędkość silników z zawisu). Panikuje jeśli wejście nie jest wariantem `Quadrotor`.

### Struct `FixedWingMixer`

Mikser dla samolotu (fixed-wing).

```rust
pub struct FixedWingMixer {
    cruise_throttle: f64,
}
```

Przelicza `AttitudeCommand` bezpośrednio na `KnownActuatorInput::FixedWing` z clampowaniem wartości do zakresów: throttle [0, 1], aileron/elevator/rudder [-1, 1].

#### Metody

- **`FixedWingMixer::new(cruise_throttle: f64) -> Self`**
  Tworzy mikser z podaną przepustnicą przelotową.

- **`FixedWingMixer::from_equilibrium(input: KnownActuatorInput) -> Self`**
  Tworzy mikser na podstawie wejścia równowagi. Panikuje jeśli wejście nie jest wariantem `FixedWing`.

---

## 8. Moduł `cascade`

Trzypoziomowy kaskadowy regulator lotu z pełnym sterowaniem XYZ + yaw.

### Struct `CascadeController<Pz, Pxy, I>`

```rust
pub struct CascadeController<Pz, Pxy, I>
where
    Pz:  VelocityProfiler,
    Pxy: VelocityProfiler,
    I:   InnerLoop,
```

#### Parametry generyczne

- **`Pz`** — profiler prędkości dla osi Z (wysokość). Implementuje `VelocityProfiler`.
- **`Pxy`** — profiler prędkości dla osi XY (poziom). Może różnić się od `Pz` — np. `SqrtProfiler` dla Z i `LinearProfiler` dla XY, bez alokacji na stercie.
- **`I`** — implementacja pętli wewnętrznej. Implementuje `InnerLoop`.

#### Pola publiczne

- **`max_tilt_rad: f64`** — maksymalny kąt przechyłu dla sterowania XY [rad]. Domyślnie `0.15` (~8.6°). Zapobiega saturacji silników przy łącznym roll+pitch.
- **`tilt_compensation: bool`** — kompensacja utraty ciągu z powodu przechyłu kadłuba. Domyślnie `true`. Dzieli throttle przez `cos(roll) × cos(pitch)` (z dolnym ograniczeniem 0.3).

#### Pola prywatne (konfigurowane przez konstruktor)

- `profiler_z` / `profiler_xy` — profilery prędkości (pętla zewnętrzna).
- `vel_loop_z`, `vel_loop_x`, `vel_loop_y` — pętle prędkości (pętla środkowa): vZ → delta throttle, vX → target pitch, vY → target roll.
- `att_loop_roll`, `att_loop_pitch`, `att_loop_yaw` — pętle kątowe (pętla wewnętrzna).
- `mixer: Box<dyn Mixer>` — mikser aktuatorów.

#### Kaskada sterowania

Algorytm `update()` realizuje trzy poziomy kaskady:

1. **Pozycja → prędkość** (pętla zewnętrzna): Błąd pozycji na każdej aktywnej osi przechodzi przez odpowiedni profiler (`profiler_z` dla Z, `profiler_xy` dla XY), dając zadaną prędkość. Osie z `None` w `FlightTarget` produkują zerową komendę prędkości.

2. **Prędkość → kąty / throttle** (pętla środkowa): Błąd prędkości przechodzi przez pętle PID:
   - vZ → delta throttle (dodawane do throttle równowagi)
   - vX → target pitch (clampowany do `±max_tilt_rad`)
   - vY → target roll (z negacją — pozytywny roll daje ciąg w -Y)

3. **Kąty → silniki** (pętla wewnętrzna): Błąd kątowy (target − aktualny euler) przechodzi przez pętle PID, a wynik trafia do miksera jako `AttitudeCommand`.

Kompensacja przechyłu: jeśli `tilt_compensation = true`, throttle jest dzielony przez `cos(roll) × cos(pitch)`, aby utrzymać stałą siłę pionową niezależnie od pochylenia drona.

#### Metody

- **`CascadeController::new(mixer, profiler_z, profiler_xy, vel_loop_z, vel_loop_x, vel_loop_y, att_loop_roll, att_loop_pitch, att_loop_yaw) -> Self`**
  Konstruktor przyjmujący wszystkie komponenty kaskady. Ustawia `max_tilt_rad = 0.15` i `tilt_compensation = true`.

### Funkcja `make_cascade`

```rust
pub fn make_cascade(model: &dyn VehicleModel)
    -> CascadeController<SqrtProfiler, SqrtProfiler, PidLoop>
```

Funkcja fabryczna tworząca regulator kaskadowy z domyślnymi parametrami PID na podstawie modelu pojazdu. Automatycznie dobiera mikser (`QuadrotorMixer` lub `FixedWingMixer`) na podstawie typu wejścia równowagi modelu.

Domyślne strojenie PID:
- Prędkość Z: `PidLoop(0.3, 0.1, 0.0, 0.45, 0.45)`
- Prędkość X/Y: `PidLoop(0.4, 0.05, 0.0, 0.5, 0.35)`
- Kąt roll/pitch: `PidLoop(4.0, 0.0, 0.2, 1.0, 1.0)`
- Kąt yaw: `PidLoop(2.0, 0.1, 0.0, 0.5, 0.5)`

---

## 9. Moduł `lqr`

Regulator liniowo-kwadratowy (LQR) i liniowo-kwadratowy z całkowaniem (LQI). Obejmuje linearyzację modelu, rozwiązywanie ciągłego algebraicznego równania Riccatiego (CARE) oraz projektowanie regulatora.

### 9.1. Podmoduł `linearize`

Linearyzacja numeryczna modelu pojazdu wokół punktu pracy.

#### Struct `LinearizedModel`

```rust
#[derive(Debug, Clone)]
pub struct LinearizedModel {
    pub a:  DMatrix<f64>,   // macierz dynamiki stanu [13×13]
    pub b:  DMatrix<f64>,   // macierz wejścia [13×m]
    pub x0: DVector<f64>,   // punkt pracy — wektor stanu
    pub u0: DVector<f64>,   // punkt pracy — wektor sterowania
}
```

#### Pola

- **`a: DMatrix<f64>`** — macierz dynamiki stanu A. Rozmiar 13×13 (13, a nie 12, ponieważ kwaternion ma 4 składowe przy 3 stopniach swobody).
- **`b: DMatrix<f64>`** — macierz wejścia B. Rozmiar 13×m, gdzie m = liczba wejść (4 dla quadrotora).
- **`x0: DVector<f64>`** — wektor stanu w punkcie pracy (linearyzacji).
- **`u0: DVector<f64>`** — wektor sterowania w punkcie pracy (równowaga).

#### Funkcja `linearize`

```rust
pub fn linearize(
    model: &dyn VehicleModel,
    state0: &DroneState,
    input0: &KnownActuatorInput,
) -> LinearizedModel
```

Numeryczna linearyzacja modelu nieliniowego wokół stanu `state0` i wejścia `input0`. Oblicza macierze A i B metodą różnic centralnych (central finite differences) z krokiem ε = 1.49×10⁻⁸.

#### Funkcje konwersji stanu

- **`state_to_vec(state: &DroneState) -> DVector<f64>`**
  Konwertuje `DroneState` na wektor 13D: `[x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]`.

- **`vec_to_state(vec: &DVector<f64>, template: &DroneState) -> DroneState`**
  Konwertuje wektor 13D z powrotem na `DroneState`. `template` dostarcza pola nie zawarte w wektorze (np. `actuator_state`).

- **`input_to_vec(input: &KnownActuatorInput) -> DVector<f64>`**
  Konwertuje wejście aktuatorów na wektor. Dla quadrotora: `[FR, FL, RL, RR]`. Dla fixed-wing: `[throttle, aileron, elevator, rudder]`.

- **`vec_to_input(v: &DVector<f64>, template: &KnownActuatorInput) -> KnownActuatorInput`**
  Konwertuje wektor z powrotem na `KnownActuatorInput`. `template` determinuje wariant wyjścia.

#### Funkcje dyskretyzacji

- **`discretize_euler(a: &DMatrix<f64>, b: &DMatrix<f64>, dt: f64) -> (DMatrix<f64>, DMatrix<f64>)`**
  Dyskretyzacja jawnym (forward) Eulerem: `Ad = I + A·dt`, `Bd = B·dt`. Tania i dokładna dla małych dt, ale numerycznie niestabilna dla dużych kroków czasowych (wartości własne Ad wychodzą poza koło jednostkowe).

- **`discretize_implicit_euler(a: &DMatrix<f64>, b: &DMatrix<f64>, dt: f64) -> (DMatrix<f64>, DMatrix<f64>)`**
  Dyskretyzacja niejawnym (backward/implicit) Eulerem: `Ad = (I − A·dt)⁻¹`, `Bd = Ad · B·dt`. A-stabilna — wartości własne Ad zawsze pozostają wewnątrz koła jednostkowego dla dowolnego stabilnego lub marginalnie stabilnego systemu ciągłego i dowolnego dt > 0. Bezpieczna do użycia z dużymi krokami predykcji.

### 9.2. Podmoduł `care`

Rozwiązywanie ciągłego algebraicznego równania Riccatiego (CARE) dla LQR.

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

#### Warianty

- **`WrongDimensionsA`** — macierz A nie jest kwadratowa.
- **`WrongDimensionsB`** — B ma niepoprawną liczbę wierszy (powinna być równa wymiarowi A).
- **`WrongDimensionsQ`** — Q ma niepoprawne wymiary (powinna być n×n).
- **`WrongDimensionsR`** — R ma niepoprawne wymiary (powinna być m×m).
- **`SingularR`** — macierz R jest osobliwa (nieodwracalna). Elementy diagonalne muszą być dodatnie.
- **`SingularLyapunov`** — układ Lyapunova osobliwy (zerowa wartość własna pętli zamkniętej utrzymuje się po regularyzacji).
- **`NotConverged`** — CARE nie zbiegło po `max_iter` iteracjach Newtona. Pole `residual` zawiera końcowe residuum.

#### Struct `SolverParams`

```rust
#[derive(Debug, Clone)]
pub struct SolverParams {
    pub max_iter:  usize,
    pub tolerance: f64,
}
```

- **`max_iter: usize`** — maksymalna liczba iteracji. Domyślnie `1000`.
- **`tolerance: f64`** — tolerancja zbieżności. Domyślnie `1e-8`.

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

- **`p: DMatrix<f64>`** — macierz rozwiązania P równania CARE.
- **`k: DMatrix<f64>`** — macierz wzmocnień K = R⁻¹BᵀP.
- **`flow_steps: usize`** — liczba kroków przepływu Riccatiego RK4 (faza 1).
- **`newton_iters: usize`** — liczba iteracji Newton-Kleinmana (faza 2). `0` oznacza, że faza 1 wystarczyła.
- **`care_residual: f64`** — końcowe residuum równania CARE.

#### Funkcja `solve_care`

```rust
pub fn solve_care(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    q: &DMatrix<f64>,
    r: &DMatrix<f64>,
    params: &SolverParams,
) -> Result<RiccatiSolution, CareError>
```

Rozwiązuje ciągłe algebraiczne równanie Riccatiego: `AᵀP + PA − PBR⁻¹BᵀP + Q = 0`.

Algorytm dwufazowy:
1. **Faza 1**: Całkowanie ODE Riccatiego metodą RK4 do uzyskania dobrego przybliżenia P.
2. **Faza 2**: Udoskonalenie Newton-Kleinmana (iteracyjne rozwiązywanie równań Lyapunova).

Automatycznie obsługuje „martwe" stany — kierunki stanu zdekuplowane w punkcie pracy (np. składowa w kwaternionu przy zawisie). Dla tych kierunków wagi Q są zerowane, co daje fizycznie poprawne K = 0.

#### Funkcje pomocnicze

- **`build_q_diagonal(weights: &[f64]) -> DMatrix<f64>`**
  Buduje diagonalną macierz Q z wektora wag. Rozmiar wynikowej macierzy: n×n, gdzie n = długość `weights`.

- **`build_r_diagonal(weights: &[f64]) -> DMatrix<f64>`**
  Buduje diagonalną macierz R z wektora wag. Rozmiar wynikowej macierzy: m×m, gdzie m = długość `weights`.

### 9.3. Podmoduł `lqr`

Regulator LQR — stabilizacja wokół ustalonego punktu pracy.

#### Struct `LqrController`

```rust
pub struct LqrController {
    // pola prywatne: k, x0, u0, input_template, u_limits
}
```

Regulator LQR zaprojektowany offline dla jednego punktu pracy (stanu równowagi). Stabilizuje dron wokół tego punktu niezależnie od `FlightTarget`. Nie śledzi dowolnych celów — do śledzenia służy `LqiController`.

#### Metody

- **`LqrController::design(model: &dyn VehicleModel, trim_state: &DroneState, q_weights: &[f64], r_weights: &[f64], u_limits: Vec<(f64, f64)>) -> Result<Self, CareError>`**
  Projektuje regulator LQR wokół stanu `trim_state`:
  - `model` — model pojazdu (używany do linearyzacji i pobrania wejścia równowagi).
  - `trim_state` — stan zawisu/lotu ustalonego wokół którego następuje linearyzacja.
  - `q_weights` — wagi diagonalne macierzy Q (13 elementów dla quadrotora: pozycja, prędkość, prędkość kątowa, kwaternion).
  - `r_weights` — wagi diagonalne macierzy R (4 elementy dla quadrotora: po jednym na silnik).
  - `u_limits` — ograniczenia wejść sterujących `[(min, max); m]`.
  Zwraca `Err(CareError)` jeśli solver CARE nie zbiegnie.

- **`compute_control(&self, state: &DroneState) -> DVector<f64>`**
  Oblicza wektor sterowania: `u = u₀ − K·(x − x₀)`, z clampowaniem do `u_limits`.

Implementuje `Controller`: metoda `update()` ignoruje `target` i `dt` — LQR zawsze stabilizuje wokół punktu projektowego.

### 9.4. Podmoduł `lqi`

Regulator LQI — rozszerzenie LQR o 4 stany całkowe eliminujące uchyb ustalony.

#### Enum `LqiError`

```rust
#[derive(Debug, Error)]
pub enum LqiError {
    WrongCIntShape { n_integrals: usize, n_plant: usize, rows: usize, cols: usize },
    WrongQWeightsLen { expected: usize, n_plant: usize, n_integrals: usize, actual: usize },
    Care(CareError),
}
```

- **`WrongCIntShape`** — macierz `c_int` ma niepoprawne wymiary (oczekiwane 4×n_plant).
- **`WrongQWeightsLen`** — `q_weights` ma niepoprawną długość (oczekiwane n_plant + 4).
- **`Care(CareError)`** — błąd solvera CARE.

#### Struct `LqiController`

```rust
pub struct LqiController {
    // pola prywatne: k, x0, u0, xi, input_template, u_limits
    pub xi_limits: [f64; 4],
}
```

Rozszerzony stan: `z = [δx (13D odchylenie od punktu pracy); ξ (4D całki)]` — łącznie 17D.

Macierz wzmocnień K ∈ ℝ^(m×17) jest obliczana jednokrotnie przez CARE na systemie rozszerzonym i nie zmienia się w trakcie działania.

#### Pole publiczne

- **`xi_limits: [f64; 4]`** — ograniczenia anti-windup dla każdego integratora [m·s, m·s, m·s, rad·s]. Domyślne wartości: `[5.0, 5.0, 2.0, 2π]`.

#### Metody

- **`LqiController::design(model: &dyn VehicleModel, trim_state: &DroneState, c_int: DMatrix<f64>, q_weights: &[f64], r_weights: &[f64], u_limits: Vec<(f64, f64)>) -> Result<Self, LqiError>`**
  Projektuje regulator LQI:
  - `c_int` — macierz selekcji wyjść (4×n_plant). Mapuje stany modelu na całkowane wyjścia. Dla standardowej konfiguracji quadrotora użyj `quadrotor_c_integral(n_plant)`.
  - `q_weights` — musi mieć długość `n_plant + 4` (17 dla quadrotora): indeksy 0..n_plant to wagi odchyleń stanu, indeksy n_plant.. to wagi całek [ξ_x, ξ_y, ξ_z, ξ_ψ]. Typowe wagi całkowe: 5–50.
  - Pozostałe parametry jak w `LqrController::design`.

  Buduje system rozszerzony:
  ```
  A_aug = [A  0]     B_aug = [B]
          [-C 0]              [0]
  ```
  i rozwiązuje CARE na tym systemie.

- **`update_integrals(&mut self, state: &DroneState, target: &FlightTarget, dt: f64)`** *(prywatna)*
  Aktualizuje stany całkowe dla aktywnych osi `FlightTarget`. Osie z `None` mają zamrożone integratory (ξ̇ = 0). Stosuje clampowanie anti-windup do `xi_limits`.

Implementuje `Controller`:
- `update()` — aktualizuje całki, oblicza `u = u₀ − K·z` i zwraca sygnały aktuatorów.
- `reset()` — zeruje wszystkie 4 integratory.

#### Funkcja `quadrotor_c_integral`

```rust
pub fn quadrotor_c_integral(n_plant: usize) -> DMatrix<f64>
```

Zwraca standardową macierz selekcji C ∈ ℝ^(4×n_plant) dla quadrotora:
- ξ_x ← x (indeks stanu 0)
- ξ_y ← y (indeks stanu 1)
- ξ_z ← z (indeks stanu 2)
- ξ_ψ ← 2·qz (indeks stanu 11; linearyzacja yaw wokół kwaternionu jednostkowego: d(yaw)/d(qz)|_{q=I} = 2)

---

## 10. Moduł `trajectory`

Generatory trajektorii zmiennych w czasie do śledzenia ścieżek w otwartej pętli.

### Trait `Trajectory`

```rust
pub trait Trajectory: Send + Sync {
    fn target(&self, time_s: f64) -> FlightTarget;
}
```

#### Metody

- **`target(&self, time_s: f64) -> FlightTarget`**
  Oblicza cel lotu w danej chwili czasu symulacji [s]. Wywoływana co krok symulacji.

### Struct `HoldTrajectory`

```rust
#[derive(Debug, Clone)]
pub struct HoldTrajectory {
    pub inner: FlightTarget,
}
```

Zawsze zwraca ten sam cel — wrapper no-op dla stałego punktu.

#### Pola

- **`inner: FlightTarget`** — stały cel zwracany dla każdej chwili czasu.

### Struct `WaypointTrajectory`

```rust
#[derive(Debug, Clone)]
pub struct WaypointTrajectory {
    // prywatne: waypoints: Vec<(f64, FlightTarget)>
}
```

Trajektoria odcinkowo-liniowa przez punkty nawigacyjne z czasem.

#### Zachowanie interpolacji

- Waypoints: `Vec<(time_s, FlightTarget)>` posortowane rosnąco po czasie.
- **Przed pierwszym waypointem** → utrzymywany jest pierwszy waypoint.
- **Po ostatnim waypoincie** → utrzymywany jest ostatni waypoint.
- **Pomiędzy waypointami** → liniowa interpolacja osi z `Some`. Dla osi:
  - Obie `Some` → interpolacja liniowa (lerp).
  - Tylko jedna `Some` → utrzymywana wartość z tej jednej strony.
  - Obie `None` → wynik `None`.

#### Metody

- **`WaypointTrajectory::new(wps: Vec<(f64, FlightTarget)>) -> Self`**
  Tworzy trajektorię z listy par `(czas_s, FlightTarget)`. Lista jest sortowana po czasie. **Panikuje** jeśli `wps` jest pusty.

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

Orbita kołowa w płaszczyźnie poziomej na stałej wysokości.

#### Pola

- **`cx: f64`** — współrzędna X środka orbity [m].
- **`cy: f64`** — współrzędna Y środka orbity [m].
- **`radius: f64`** — promień orbity [m].
- **`omega: f64`** — prędkość kątowa [rad/s]; dodatnia = CCW (przeciwnie do ruchu wskazówek zegara).
- **`altitude_m: f64`** — stała wysokość lotu [m].

Pozycja w chwili t: `x = cx + radius·cos(ω·t)`, `y = cy + radius·sin(ω·t)`, `z = altitude_m`.
