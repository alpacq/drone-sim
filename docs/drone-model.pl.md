# `drone-model` — dokumentacja referencyjna

## 1. Przegląd

Crate `drone-model` dostarcza modele fizyczne pojazdów latających (quadrotor, samolot F-16) oraz infrastrukturę potrzebną do symulacji lotu z sześcioma stopniami swobody (6-DOF). Zawiera:

- Reprezentację stanu drona (`DroneState`) z pozycją, prędkością, orientacją (kwaternion) i stanem aktuatorów.
- Krok czasowy z walidacją (`TimeStep`).
- Indeksowaną tablicę czterech silników quadrotora (`Motor`, `MotorArray<T>`).
- Interfejsy modelu pojazdu (`VehicleModel`, `AeroModel`) i wspólne struktury sił/momentów.
- Dynamikę ciała sztywnego 6-DOF (`dynamics_6dof`).
- Pełny model quadrotora (parametry DJI Mini 3) z wirnikami i oporem aerodynamicznym.
- Model samolotu F-16A z aerodynamiką tablicową, silnikiem odrzutowym i solverem trymowania.
- Modele atmosfery (ISA, stała gęstość) i konwersje kątów Eulera ↔ kwaternion.

---

## 2. Moduł `state`

### `ActuatorState`

Wewnętrzny stan aktuatorów przechowywany w `DroneState`, dzięki czemu każdy snapshot stanu jest samowystarczalny.

```rust
pub enum ActuatorState {
    QuadrotorMotors(MotorArray<f64>),
    FixedWingEngine {
        current_throttle: f64,
        current_thrust_n: f64,
    },
}
```

**Warianty:**

- `QuadrotorMotors(MotorArray<f64>)` — prędkości obrotowe czterech silników [rad/s] w konfiguracji X-frame.
- `FixedWingEngine { current_throttle, current_thrust_n }` — stan silnika odrzutowego: przefiltrowane ustawienie przepustnicy [0, 1] i wynikowy ciąg w niutonach.

### `DroneState`

Pełny stan drona w chwili *t*. Wszystkie wartości w układzie świata (ENU), z wyjątkiem `angular_velocity` (układ ciała).

```rust
pub struct DroneState {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub actuator_state: Option<ActuatorState>,
}
```

**Pola:**

| Pole | Typ | Opis |
|------|-----|------|
| `position` | `Vector3<f64>` | Pozycja [x, y, z] w metrach; oś z skierowana w górę. |
| `velocity` | `Vector3<f64>` | Prędkość liniowa [vx, vy, vz] w m/s, układ świata. |
| `angular_velocity` | `Vector3<f64>` | Prędkość kątowa [p, q, r] w rad/s, **układ ciała**. |
| `orientation` | `UnitQuaternion<f64>` | Orientacja jako kwaternion jednostkowy — rotacja z układu świata do ciała. |
| `actuator_state` | `Option<ActuatorState>` | Opcjonalny stan aktuatorów (silniki quadrotora lub silnik odrzutowy). |

**Metody:**

#### `euler_angles(&self) -> EulerAngles`

Zwraca orientację jako kąty Eulera (konwencja ZYX). Przeznaczona do wizualizacji i porównań z telemetrią DJI.

#### `on_ground() -> Self`

Konstruktor tworzący stan zerowy (pozycja, prędkość, prędkość kątowa = 0; orientacja = tożsamość; brak stanu aktuatorów). Używany jako punkt startowy symulacji.

---

## 3. Moduł `time`

### `TimeStep`

Newtype opakowujący krok czasowy symulacji (`f64`). Gwarantuje, że dt > 0.

```rust
pub struct TimeStep(f64);
```

**Metody:**

#### `new(dt: f64) -> Result<Self, TimeStepError>`

Tworzy krok czasowy. Zwraca `Err(TimeStepError)` jeśli `dt <= 0`.

- `dt` — krok czasowy w sekundach.

#### `constant(dt: f64) -> Self`

Tworzy krok czasowy, panikując jeśli `dt <= 0`. Używać tylko dla stałych znanych w czasie kompilacji.

#### `seconds(self) -> f64`

Zwraca wartość kroku w sekundach.

#### `half(self) -> Self`

Zwraca połowę kroku czasowego. Przydatne dla integratorów typu RK2/RK4.

### `TimeStepError`

Błąd zwracany przy próbie utworzenia kroku czasowego z wartością ≤ 0.

```rust
pub struct TimeStepError(f64);
```

Implementuje `Display` (komunikat: „TimeStep must be positive, got {wartość}") i `Error`.

---

## 4. Moduł `motor`

### `Motor`

Enum identyfikujący cztery silniki quadrotora w konfiguracji X-frame.

```rust
pub enum Motor {
    FrontRight = 0,  // CW (zgodnie z ruchem wskazówek)
    FrontLeft  = 1,  // CCW
    RearLeft   = 2,  // CW
    RearRight  = 3,  // CCW
}
```

**Warianty (widok z góry):**

- `FrontRight` (indeks 0) — prawy przedni, obraca się zgodnie ze wskazówkami zegara (CW).
- `FrontLeft` (indeks 1) — lewy przedni, obraca się przeciwnie do wskazówek (CCW).
- `RearLeft` (indeks 2) — lewy tylny, CW.
- `RearRight` (indeks 3) — prawy tylny, CCW.

**Stałe:**

- `ALL: [Motor; 4]` — tablica wszystkich wariantów w kolejności indeksów.

**Metody:**

#### `is_clockwise(self) -> bool`

Zwraca `true` dla silników CW (`FrontRight`, `RearLeft`), `false` dla CCW. Kierunki obrotów parują się tak, aby w zawisie moment odchylenia (yaw) wynosił zero.

### `MotorArray<T>`

Tablica czterech wartości indeksowana wariantami `Motor`. Generyczna — może przechowywać prędkości (`f64`), siły, momenty itp.

```rust
pub struct MotorArray<T>([T; 4]);
```

**Metody (dostępne dla wszystkich `T`):**

#### `new(front_right: T, front_left: T, rear_left: T, rear_right: T) -> Self`

Tworzy tablicę z wartościami w kolejności: FrontRight, FrontLeft, RearLeft, RearRight.

#### `iter(&self) -> impl Iterator<Item = (Motor, &T)>`

Iterator zwracający pary `(Motor, &T)` dla wszystkich czterech silników.

**Metody (wymagają `T: Copy`):**

#### `uniform(value: T) -> Self`

Tworzy tablicę z taką samą wartością dla każdego silnika.

#### `map<U, F: Fn(T) -> U>(&self, f: F) -> MotorArray<U>`

Aplikuje funkcję `f` do każdego elementu, zwracając nową tablicę.

#### `map_with_motor<U, F: Fn(Motor, T) -> U>(&self, f: F) -> MotorArray<U>`

Jak `map`, ale funkcja otrzymuje również identyfikator silnika.

#### `sum(self) -> T` (wymaga `T: Add<Output = T>`)

Sumuje cztery elementy tablicy.

**Implementacje trait:**

- `Index<Motor>` / `IndexMut<Motor>` — dostęp do elementów przez `arr[Motor::FrontRight]`.
- `From<[T; 4]>` / `Into<[T; 4]>` — konwersja z/do zwykłej tablicy.

---

## 5. Moduł `vehicle`

### `KnownActuatorInput`

Wejście sterowania dla pojazdu.

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

**Warianty:**

- `Quadrotor(MotorArray<f64>)` — komenda prędkości obrotowej [ω₀, ω₁, ω₂, ω₃] w rad/s.
- `FixedWing { throttle, aileron, elevator, rudder }`:
  - `throttle` — przepustnica [0, 1].
  - `aileron` — lotki (przechylenie) [−1, 1].
  - `elevator` — ster wysokości (pochylenie) [−1, 1].
  - `rudder` — ster kierunku (odchylenie) [−1, 1].

### `ForcesAndMoments`

Siły i momenty działające na pojazd w układzie ciała.

```rust
pub struct ForcesAndMoments {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}
```

- `force` — siła wypadkowa [N] w układzie ciała.
- `torque` — moment wypadkowy [N·m] w układzie ciała.

Implementuje `Add` (sumowanie sił i momentów) oraz `Default` (zerowe siły/momenty).

#### `new(force: Vector3<f64>, torque: Vector3<f64>) -> Self`

Konstruktor.

### `StateDot`

Pochodne stanu po czasie — wynik funkcji dynamiki.

```rust
pub struct StateDot {
    pub velocity: Vector3<f64>,
    pub acceleration: Vector3<f64>,
    pub angular_acceleration: Vector3<f64>,
    pub orientation_dot: Quaternion<f64>,
}
```

- `velocity` — ṗ = v (prędkość → pochodna pozycji).
- `acceleration` — v̇ = F/m + g (przyspieszenie liniowe w układzie świata).
- `angular_acceleration` — ω̇ = I⁻¹(τ − ω×Iω) (przyspieszenie kątowe w układzie ciała).
- `orientation_dot` — q̇ = ½·q⊗ω (pochodna kwaternionu orientacji).

### Trait `AeroModel`

Interfejs modelu aerodynamicznego.

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

Oblicza siły i momenty aerodynamiczne dla danego stanu, sterowania i warunków atmosferycznych. Wynik w układzie ciała.

### Trait `VehicleModel`

Główny interfejs modelu pojazdu latającego.

```rust
pub trait VehicleModel: Send + Sync { ... }
```

**Metody wymagane:**

#### `derivatives(&self, state: &DroneState, input: &KnownActuatorInput) -> StateDot`

Oblicza pochodne stanu (dstate/dt) dla danego stanu i sterowania. Czysta funkcja — brak efektów ubocznych.

#### `equilibrium_input(&self) -> KnownActuatorInput`

Zwraca sterowanie równowagowe (np. zawis dla quadrotora, lot poziomy dla samolotu).

#### `name(&self) -> &str`

Zwraca czytelną nazwę modelu (np. `"QuadrotorModel (X-frame)"`).

#### `actuator_count(&self) -> usize`

Liczba aktuatorów (4 dla quadrotora i F-16).

#### `mass(&self) -> f64`

Masa pojazdu [kg].

**Metody z domyślną implementacją:**

#### `step_actuators(&self, state: &mut DroneState, input: &KnownActuatorInput, dt: TimeStep)`

Aktualizuje stan aktuatorów (np. filtr pierwszego rzędu dla silników). Domyślna implementacja: no-op.

#### `gravity(&self) -> f64`

Przyspieszenie grawitacyjne [m/s²]. Domyślnie `9.80665`.

#### `clone_box(&self) -> Box<dyn VehicleModel>`

Klonuje model na stertę jako obiekt trait. Domyślna implementacja panikuje — każdy konkretny model powinien ją nadpisać. Używane przez fabryki kontrolerów potrzebujące własnej kopii modelu do linearyzacji.

---

## 6. Moduł `vehicle::dynamics_6dof`

### `RigidBodyParams`

Parametry ciała sztywnego: masa i tensor bezwładności z odwrotnością.

```rust
pub struct RigidBodyParams {
    pub mass: f64,
    pub inertia: Matrix3<f64>,
    pub inertia_inv: Matrix3<f64>,
}
```

- `mass` — masa [kg].
- `inertia` — tensor bezwładności [kg·m²] (macierz 3×3).
- `inertia_inv` — odwrotność tensora bezwładności (obliczana w konstruktorze).

**Metody:**

#### `new(mass: f64, ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> Self`

Tworzy parametry z pełnym tensorem bezwładności. Elementy pozadiagonalne wchodzą ze znakiem ujemnym (konwencja: macierz bezwładności ma ujemne iloczyny dewiacyjne). Panikuje jeśli tensor jest nieoddwracalny.

#### `symmetric(mass: f64, ixx: f64, iyy: f64, izz: f64) -> Self`

Tworzy parametry dla pojazdu symetrycznego (Ixy = Ixz = Iyz = 0). Typowe dla quadrotorów.

### `dynamics_6dof`

Główna funkcja dynamiki 6-DOF ciała sztywnego.

```rust
pub fn dynamics_6dof(
    state: &DroneState,
    fm: &ForcesAndMoments,
    params: &RigidBodyParams,
    gravity: f64,
) -> StateDot
```

**Parametry:**

- `state` — aktualny stan drona.
- `fm` — siły i momenty w układzie ciała.
- `params` — parametry ciała sztywnego (masa, tensor bezwładności).
- `gravity` — przyspieszenie grawitacyjne [m/s²].

**Zwraca:** `StateDot` — pochodne stanu po czasie.

**Fizyka (trzy etapy):**

1. **Translacja** — siły z układu ciała transformowane do świata kwaternionenem orientacji, dodana grawitacja (ENU: oś z w dół):
   - `F_world = R(q) · F_body`
   - `a = F_world / m + [0, 0, -g]`

2. **Rotacja** — równanie Eulera: `I·ω̇ = τ − ω×(I·ω)`. Człon `ω×(I·ω)` to efekt żyroskopowy ciała sztywnego — rotacja zmienia kierunek momentu pędu, generując dodatkowy moment. Bez tego członu symulacja byłaby niestabilna przy dużych prędkościach kątowych.

3. **Pochodna kwaternionu** — `q̇ = ½·q⊗ω`, gdzie ω jest zapisane jako kwaternion z częścią skalarną 0. Po całkowaniu kwaternion wymaga renormalizacji (operacja algebraiczna nie zachowuje |q| = 1).

---

## 7. Moduł `vehicle::quadrotor`

### `QuadrotorParams`

Stałe fizyczne quadrotora — ładowane z pliku TOML lub tworzone konstruktorem.

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

**Pola:**

- `mass` — masa [kg].
- `arm_length` — długość ramienia (od środka masy do silnika) [m].
- `k_thrust` — współczynnik ciągu: F = k_thrust · ω² [N·s²/rad²].
- `k_torque` — współczynnik momentu obrotowego: τ = k_torque · ω² [N·m·s²/rad²].
- `k_drag` — współczynnik oporu aerodynamicznego ciała: F_drag = k_drag · v² [kg/m]. Opór izotropowy, kwadratowy, przeciwny do wektora prędkości. Prędkość graniczna: v_t = √(m·g / k_drag).
- `rigid_body` — parametry ciała sztywnego (`RigidBodyParams`).

**Metody:**

#### `new(mass, arm_length, k_thrust, k_torque, k_drag, ixx, iyy, izz) -> Self`

Konstruktor. Tworzy `RigidBodyParams::symmetric` z podanych momentów bezwładności.

#### `mini3() -> Self`

Zwraca parametry odpowiadające DJI Mini 3 (masa 0.249 kg, ramię 0.085 m). Zawiera szczegółowe wyprowadzenie momentów bezwładności z masy silników i kadłuba.

### `QuadrotorAero`

Model aerodynamiczny quadrotora. Implementuje trait `AeroModel`.

```rust
pub struct QuadrotorAero {
    pub params: QuadrotorParams,
}
```

Oblicza:
- **Ciąg** każdego silnika: F = k_thrust · ω².
- **Moment obrotowy** każdego silnika: τ = k_torque · ω².
- **Siłę wypadkową** (suma ciągów wzdłuż osi z ciała + opór aerodynamiczny).
- **Momenty**: przechylenie (roll) z różnicy ciągów lewo–prawo, pochylenie (pitch) z różnicy tył–przód, odchylenie (yaw) z różnicy momentów CW–CCW.

### `QuadrotorModel`

Pełny model quadrotora — implementuje trait `VehicleModel`.

```rust
pub struct QuadrotorModel {
    pub params: QuadrotorParams,
    pub rotors: QuadrotorRotors,
    pub aero: QuadrotorAero,
    pub atmosphere: Box<dyn AtmosphereModel>,
}
```

**Metody:**

#### `new(params: QuadrotorParams, rotors: QuadrotorRotors, atmosphere: Box<dyn AtmosphereModel>) -> Self`

Konstruktor ogólny.

#### `mini3() -> Self`

Model DJI Mini 3 z atmosferą ISA i wirnikami w stanie zawisu. Prędkość zawisu obliczana analitycznie: ω = √(m·g / (4·k_thrust)).

#### `mini3_simple() -> Self`

Uproszczony Mini 3 ze stałą gęstością powietrza i wirnikami od zera. Przydatny do szybkich testów.

**Implementacja `VehicleModel`:**

- `derivatives` — oblicza siły aerodynamiczne, dodaje moment żyroskopowy wirników, wywołuje `dynamics_6dof`.
- `step_actuators` — filtr pierwszego rzędu na prędkościach silników: `ω_new = α·ω_cur + (1−α)·ω_cmd`, gdzie `α = exp(−dt/τ)`. Ogranicza prędkość do `[min_speed, max_speed]`.
- `equilibrium_input` — zwis: ω = √(m·g / (4·k_thrust)) dla każdego silnika.
- `name` → `"QuadrotorModel (X-frame)"`.
- `actuator_count` → `4`.
- `mass` → `params.mass`.
- `clone_box` — klonuje pełny model (wraz z atmosferą przez `clone_box`).

### `RotorParams`

Parametry dynamiki wirnika.

```rust
pub struct RotorParams {
    pub time_constant_s: f64,
    pub rotor_inertia: f64,
    pub max_speed: f64,
    pub min_speed: f64,
}
```

- `time_constant_s` — stała czasowa pierwszego rzędu [s] (jak szybko silnik reaguje na komendę).
- `rotor_inertia` — moment bezwładności wirnika [kg·m²].
- `max_speed` — maksymalna prędkość obrotowa [rad/s].
- `min_speed` — minimalna prędkość obrotowa [rad/s].

#### `mini3() -> Self`

Parametry wirnika Mini 3: τ = 0.04 s, J = 2.0e-5 kg·m², max 1120 rad/s.

### `QuadrotorRotors`

Zarządzanie stanem czterech wirników z dynamiką pierwszego rzędu.

```rust
pub struct QuadrotorRotors {
    pub params: RotorParams,
    current_speeds: MotorArray<f64>,
}
```

**Metody:**

#### `new(params: RotorParams) -> Self`

Tworzy wirniki z prędkościami początkowymi = 0.

#### `mini3() -> Self`

Skrót: `Self::new(RotorParams::mini3())`.

#### `at_hover(params: RotorParams, hover_speed: f64) -> Self`

Tworzy wirniki z prędkościami początkowymi ustawionymi na wartość zawisu.

#### `speeds(&self) -> &MotorArray<f64>`

Zwraca aktualne prędkości obrotowe.

#### `step(&mut self, commanded: &MotorArray<f64>, dt: TimeStep)`

Wykonuje jeden krok dynamiki wirników (filtr pierwszego rzędu). Ogranicza prędkości do `[min_speed, max_speed]`.

#### `gyroscopic_torque(&self, aircraft_angular_velocity: &Vector3<f64>) -> Vector3<f64>`

Oblicza moment żyroskopowy wirników: `τ_gyro = ω_aircraft × h_rotors`, gdzie `h_rotors = [0, 0, J_r · Σ(σ_i · ω_i)]`. Silniki CW mają σ = +1, CCW mają σ = −1. W symetrycznym zawisie (wszystkie ω równe) sumaryczny moment pędu wirników jest zerowy (CW i CCW się znoszą).

### `body_drag`

```rust
pub fn body_drag(velocity_world: &Vector3<f64>, k_drag: f64) -> Vector3<f64>
```

Oblicza siłę oporu aerodynamicznego ciała w układzie świata.

- `velocity_world` — prędkość w układzie świata [m/s].
- `k_drag` — współczynnik oporu [kg/m].
- **Zwraca:** wektor siły oporu = −v̂ · k_drag · |v|² (skierowany przeciwnie do prędkości, skaluje się z kwadratem prędkości). Dla |v| < 1e-6 zwraca wektor zerowy.

---

## 8. Moduł `vehicle::fixed_wing::f16`

### `F16Params`

Parametry masowe i bezwładnościowe samolotu F-16A.

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

- `mass` — masa [kg].
- `ixx`, `iyy`, `izz` — główne momenty bezwładności [kg·m²].
- `ixy`, `ixz`, `iyz` — iloczyny dewiacyjne [kg·m²].

#### `f16a() -> Self`

Parametry F-16A wg NASA TP-1538 (masa 9295.44 kg, Ixz = 1331.4 kg·m²).

### `F16Model`

Pełny model F-16 — implementuje trait `VehicleModel`.

```rust
pub struct F16Model {
    pub params: F16Params,
    pub geom: F16GeomParams,
    pub engine: Mutex<JetEngine>,
    pub rigid_body: RigidBodyParams,
    pub atmosphere: Box<dyn AtmosphereModel>,
}
```

**Metody:**

#### `new(params, geom, engine, atmosphere) -> Self`

Konstruktor ogólny. Oblicza `RigidBodyParams` z parametrów masowych.

#### `f16a() -> Self`

Konfiguracja F-16A: parametry NASA, geometria F-16A, silnik F110 (dry), atmosfera ISA.

**Implementacja `VehicleModel`:**

- `derivatives` — oblicza `AeroState`, pobiera aktualny ciąg silnika, wywołuje `compute_aero` i `dynamics_6dof`.
- `step_actuators` — aktualizuje silnik odrzutowy (`JetEngine::step`) i zapisuje stan silnika do `DroneState::actuator_state`.
- `equilibrium_input` — przybliżone sterowanie lotu poziomego: throttle 0.5, elevator −0.06.
- `name` → `"F-16A (NASA TP-1538)"`.
- `actuator_count` → `4`.
- `mass` → `params.mass`.
- `clone_box` — tworzy nową instancję `F16Model::f16a()` (silnik `Mutex<JetEngine>` nie jest klonowany — nowy model startuje z zimnym silnikiem).

### `F16GeomParams`

Parametry geometryczne skrzydła.

```rust
pub struct F16GeomParams {
    pub wing_area: f64,
    pub wingspan: f64,
    pub mean_chord: f64,
}
```

- `wing_area` — powierzchnia skrzydła [m²].
- `wingspan` — rozpiętość skrzydeł [m].
- `mean_chord` — średnia cięciwa aerodynamiczna [m].

#### `f16a() -> Self`

Geometria F-16A: S = 27.87 m², b = 9.45 m, c̄ = 3.45 m.

### `AeroState`

Stan aerodynamiczny obliczony z `DroneState` i modelu atmosfery.

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

- `airspeed` — prędkość względem powietrza [m/s] (min. 1.0).
- `alpha_rad` — kąt natarcia α [rad] (ograniczony do [−0.5, 0.785]).
- `beta_rad` — kąt ślizgu β [rad] (ograniczony do [−0.524, 0.524]).
- `mach` — liczba Macha [−].
- `qbar` — ciśnienie dynamiczne q̄ = ½ρV² [Pa].
- `v_body` — prędkość w układzie ciała [u, v, w] [m/s].

#### `compute(state: &DroneState, atmosphere: &dyn AtmosphereModel) -> Self`

Oblicza stan aerodynamiczny z prędkości świata, orientacji i modelu atmosfery.

#### `alpha_deg(&self) -> f64` / `beta_deg(&self) -> f64`

Konwersja kątów na stopnie.

### `JetEngineParams`

Parametry silnika odrzutowego.

```rust
pub struct JetEngineParams {
    pub thrust_sl_max: f64,
    pub thrust_sl_idle: f64,
    pub time_constant_s: f64,
    pub altitude_exp: f64,
}
```

- `thrust_sl_max` — maksymalny ciąg na poziomie morza przy Mach = 0 [N].
- `thrust_sl_idle` — ciąg jałowy na poziomie morza [N].
- `time_constant_s` — stała czasowa silnika [s].
- `altitude_exp` — wykładnik skalowania ciągu z wysokością: thrust(h) ≈ thrust_sl · (ρ(h)/ρ_sl)^exp.

#### `f110_dry() -> Self`

Parametry silnika F110 (tryb suchy): 76 300 N max, 4 500 N idle, τ = 0.5 s.

### `JetEngine`

Stan silnika odrzutowego z dynamiką pierwszego rzędu.

```rust
pub struct JetEngine {
    pub params: JetEngineParams,
    pub current_throttle: f64,
    current_thrust_n: f64,
}
```

**Metody:**

#### `new(params: JetEngineParams) -> Self`

Konstruktor — silnik startuje z zerową przepustnicą i zerowym ciągiem.

#### `f110_dry() -> Self`

Skrót: `Self::new(JetEngineParams::f110_dry())`.

#### `thrust(&self) -> f64`

Aktualny ciąg silnika [N].

#### `step(&mut self, throttle_cmd: f64, altitude_m: f64, mach: f64, atmosphere: &dyn AtmosphereModel, dt: TimeStep)`

Wykonuje jeden krok dynamiki silnika:
1. Filtr pierwszego rzędu na przepustnicy (ograniczonej do [0, 1]).
2. Obliczenie ciągu z uwzględnieniem wysokości (skalowanie gęstością) i Macha (czynnik kwadratowy).
3. Interpolacja liniowa między ciągiem jałowym a maksymalnym wg aktualnej przepustnicy.

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

Oblicza siły i momenty aerodynamiczne F-16 z tablic współczynników.

- `aero` — stan aerodynamiczny (α, β, V, q̄).
- `angular` — prędkość kątowa [p, q, r] w układzie ciała [rad/s].
- `input` — sterowanie (`FixedWing` z aileron, elevator, rudder w [−1, 1]).
- `geom` — geometria skrzydła.
- `thrust_n` — aktualny ciąg silnika [N].

Sterowanie jest wewnętrznie przeliczane na stopnie wychylenia: elevator ×25°, aileron ×21.5°, rudder ×30°.

Współczynniki aerodynamiczne interpolowane z tabel 1D (α-break, β-break). Ciąg rozkładany na składowe osiowe wg kąta natarcia.

### `TrimResult`

Wynik udanego trymowania.

```rust
pub struct TrimResult {
    pub alpha_rad: f64,
    pub throttle: f64,
    pub elevator: f64,
    pub residual: f64,
}
```

- `alpha_rad` — trymowy kąt natarcia [rad].
- `throttle` — trymowa przepustnica [0, 1].
- `elevator` — trymowy ster wysokości (znormalizowany do [−1, 1]).
- `residual` — norma residuów pochodnych w punkcie trymowania [m/s²].

### `TrimError`

Błąd trymowania.

```rust
pub enum TrimError {
    NoConvergence { residual: f64, iters: usize },
}
```

- `NoConvergence` — optymalizacja nie osiągnęła akceptowalnego residuum. Zawiera wartość residuum i liczbę iteracji.

### `find_trim`

```rust
pub fn find_trim(
    _model: &F16Model,
    speed_ms: f64,
    altitude_m: f64,
) -> Result<TrimResult, TrimError>
```

Wyznacza punkt trymowania podłużnego (lot poziomy, skrzydła wypoziomowane) dla zadanej prędkości i wysokości.

- `speed_ms` — prędkość lotu [m/s].
- `altitude_m` — wysokość [m].

Szuka `(throttle, elevator, alpha)` minimalizujących sumę kwadratów przyspieszenia liniowego i kątowego metodą simpleksu Neldera-Meada (do 500 iteracji). Wewnętrznie tworzy świeże instancje `F16Model::f16a()` z rozgrzanym silnikiem.

---

## 9. Moduł `math::atmosphere`

### Trait `AtmosphereModel`

Interfejs modelu atmosfery.

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

**Metody wymagane:**

- `density(altitude_m) -> f64` — gęstość powietrza [kg/m³].
- `temperature(altitude_m) -> f64` — temperatura [K].
- `speed_of_sound(altitude_m) -> f64` — prędkość dźwięku [m/s].
- `clone_box() -> Box<dyn AtmosphereModel>` — klonowanie na stertę (potrzebne, bo modele pojazdów przechowują `Box<dyn AtmosphereModel>`).

**Metody z domyślną implementacją:**

- `dynamic_pressure(altitude_m, speed_ms) -> f64` — ciśnienie dynamiczne: q = ½ρV².
- `mach(altitude_m, speed_ms) -> f64` — liczba Macha: M = V / a. Zwraca 0 jeśli prędkość dźwięku ≤ 0.

### `Isa`

Implementacja Międzynarodowej Atmosfery Standardowej (ISA).

```rust
pub struct Isa;
```

Modeluje troposferę (0–11 000 m) z liniowym spadkiem temperatury (6.5 K/km) i stratosferę (powyżej 11 000 m) ze stałą temperaturą 216.65 K.

Stałe (moduł `isa_constants`): T₀ = 288.15 K, P₀ = 101 325 Pa, L = 0.0065 K/m, R = 287.05 J/(kg·K), g = 9.80665 m/s², γ = 1.4.

### `ConstantDensity`

Uproszczony model atmosfery ze stałą gęstością, niezależną od wysokości.

```rust
pub struct ConstantDensity {
    pub density: f64,
    pub speed_of_sound: f64,
}
```

- `density` — stała gęstość [kg/m³].
- `speed_of_sound` — stała prędkość dźwięku [m/s].

#### `sea_level() -> Self`

Warunki poziomu morza: ρ = 1.225 kg/m³, a = 340.29 m/s. Temperatura stała 288.15 K.

---

## 10. Moduł `math::euler`

### `EulerAngles`

Kąty Eulera (konwencja ZYX — yaw → pitch → roll).

```rust
pub struct EulerAngles {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}
```

- `roll` — przechylenie wokół osi X [rad].
- `pitch` — pochylenie wokół osi Y [rad].
- `yaw` — odchylenie wokół osi Z [rad].

**Metody:**

#### `new(roll: f64, pitch: f64, yaw: f64) -> Self`

Konstruktor (wartości w radianach).

#### `from_degrees(roll: f64, pitch: f64, yaw: f64) -> Self`

Konstruktor z wartościami w stopniach (konwertuje na radiany).

#### `to_degrees(&self) -> (f64, f64, f64)`

Zwraca krotkę (roll, pitch, yaw) w stopniach.

### `quat_to_euler`

```rust
pub fn quat_to_euler(q: &UnitQuaternion<f64>) -> EulerAngles
```

Konwertuje kwaternion jednostkowy na kąty Eulera (ZYX).

Wzory:
- roll = atan2(2(qw·qx + qy·qz), 1 − 2(qx² + qy²))
- pitch = asin(2(qw·qy − qz·qx)) — z clampem do [−1, 1] dla uniknięcia NaN przy błędach numerycznych
- yaw = atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))

**Uwaga:** osobliwość (gimbal lock) przy pitch = ±90°.

### `euler_to_quat`

```rust
pub fn euler_to_quat(e: &EulerAngles) -> UnitQuaternion<f64>
```

Konwertuje kąty Eulera na kwaternion jednostkowy. Rotacja: R = Rz(yaw) · Ry(pitch) · Rx(roll).

Round-trip `euler_to_quat(quat_to_euler(q))` odtwarza oryginalny kwaternion (z dokładnością numeryczną) z wyjątkiem otoczenia osobliwości pitch = ±90°.
