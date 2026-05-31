# `drone-sim` — dokumentacja referencyjna

## 1. Przegląd

Crate `drone-sim` to silnik symulacji lotu drona. Jest niezależny od konkretnego modelu pojazdu — operuje na trait `VehicleModel` z crate'a `drone-model`, dzięki czemu ten sam kod symulacyjny obsługuje quadrotory, samoloty (fixed-wing) i dowolne przyszłe konfiguracje.

Crate składa się z dwóch modułów:

- **`integrator`** — metody całkowania równań ruchu (Euler, RK4).
- **`runner`** — pętla symulacji: kontroler → aktuatory → całkowanie → zapis klatki.

Reeksporty z `lib.rs`:

```rust
pub mod integrator;
pub mod runner;
```

---

## 2. Moduł `integrator`

Moduł zawiera trait `Integrator`, funkcję pomocniczą `apply_dot` oraz dwie implementacje: `Euler` i `RK4`.

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

Interfejs metody całkowania równań ruchu drona. Trait jest object-safe (`dyn Integrator`), co pozwala wybrać metodę całkowania w runtime.

Wymagane super-traity `Send + Sync` umożliwiają użycie integratora w kontekstach wielowątkowych.

#### Metoda `step`

Wykonuje jeden krok całkowania o długości `dt`.

**Parametry:**

| Parametr | Typ | Opis |
|---|---|---|
| `model` | `&dyn VehicleModel` | Model dynamiki pojazdu (oblicza pochodne stanu). |
| `state` | `&DroneState` | Bieżący stan drona (pozycja, prędkość, orientacja, prędkość kątowa). |
| `input` | `&KnownActuatorInput` | Sygnał sterujący podany na aktuatory (np. obroty silników). |
| `dt` | `TimeStep` | Krok czasowy symulacji. |

**Zwraca:** `DroneState` — nowy stan drona po upływie `dt`.

---

### 2.2 Funkcja `apply_dot`

```rust
pub fn apply_dot(state: &DroneState, dot: &StateDot, dt: TimeStep) -> DroneState;
```

Aplikuje pochodne (`StateDot`) do bieżącego stanu, przesuwając go o `dt`. Używana wewnętrznie przez `Euler` i `RK4`.

**Operacje:**

- **Pozycja:** `position += velocity * dt`
- **Prędkość:** `velocity += acceleration * dt`
- **Prędkość kątowa:** `angular_velocity += angular_acceleration * dt`
- **Orientacja (kwaternion):** `q += orientation_dot * dt`, a następnie **renormalizacja** przez `UnitQuaternion::from_quaternion`. Renormalizacja jest konieczna, ponieważ dodawanie arytmetyczne pochodnej do kwaternionu jednostkowego narusza warunek `|q| = 1`. Bez tego kroku dryft numeryczny narastałby z każdym krokiem, powodując zniekształcenie orientacji.

**Parametry:**

| Parametr | Typ | Opis |
|---|---|---|
| `state` | `&DroneState` | Bieżący stan drona. |
| `dot` | `&StateDot` | Pochodne stanu (prędkość, przyspieszenie, przyspieszenie kątowe, pochodna orientacji). |
| `dt` | `TimeStep` | Krok czasowy. |

**Zwraca:** `DroneState` — stan po przesunięciu o `dt`. Pole `actuator_state` jest kopiowane z wejściowego stanu bez zmian.

---

### 2.3 Struktura `Euler`

```rust
pub struct Euler;
```

Metoda Eulera — najprostsza metoda całkowania numerycznego. Dokładność rzędu **O(dt)** (błąd globalny rośnie liniowo z krokiem czasowym).

Przydatna do porównań z RK4 i testowania stabilności. Dla symulacji wymagających dokładności zalecane jest użycie `RK4`.

Implementuje `Integrator`. Metoda `step` oblicza pochodne raz, a następnie wywołuje `apply_dot`.

---

### 2.4 Struktura `RK4`

```rust
pub struct RK4;
```

Metoda Rungego-Kutty 4. rzędu. Dokładność **O(dt⁴)** — standardowy wybór dla symulatorów lotu.

Algorytm oblicza cztery estymaty pochodnych (k1–k4), uśrednia je wagowo (1:2:2:1)/6, a następnie stosuje wynikową pochodną przez `apply_dot`.

Implementuje `Integrator`. Kolejne kroki:

1. `k1` — pochodne w bieżącym stanie.
2. `k2` — pochodne w stanie przesuniętym o `k1 * dt/2`.
3. `k3` — pochodne w stanie przesuniętym o `k2 * dt/2`.
4. `k4` — pochodne w stanie przesuniętym o `k3 * dt`.
5. Średnia ważona: `(k1 + 2·k2 + 2·k3 + k4) / 6`.
6. `apply_dot` z wynikową pochodną i pełnym `dt`.

---

## 3. Moduł `runner`

Moduł zawiera pętlę symulacji oraz struktury konfiguracyjne.

### 3.1 Struktura `SimFrame`

```rust
#[derive(Debug, Clone)]
pub struct SimFrame {
    pub time: f64,
    pub state: DroneState,
}
```

Pojedyncza zarejestrowana klatka symulacji.

**Pola:**

| Pole | Typ | Opis |
|---|---|---|
| `time` | `f64` | Czas symulacji w sekundach od startu. |
| `state` | `DroneState` | Pełny stan drona w danym momencie. |

---

### 3.2 Struktura `SimConfig`

```rust
pub struct SimConfig {
    pub dt: TimeStep,
    pub duration: f64,
}
```

Konfiguracja przebiegu symulacji.

**Pola:**

| Pole | Typ | Opis |
|---|---|---|
| `dt` | `TimeStep` | Stały krok czasowy symulacji. |
| `duration` | `f64` | Całkowity czas trwania symulacji w sekundach. |

Liczba kroków jest obliczana jako `ceil(duration / dt)`.

---

### 3.3 Funkcja `run`

```rust
pub fn run(
    initial_state: DroneState,
    model: &dyn VehicleModel,
    config: &SimConfig,
    integrator: &dyn Integrator,
    mut controller: impl FnMut(&DroneState, TimeStep) -> KnownActuatorInput,
) -> Vec<SimFrame>;
```

Główna funkcja symulacji. Wykonuje pętlę open-loop i zwraca pełną historię klatek.

**Parametry:**

| Parametr | Typ | Opis |
|---|---|---|
| `initial_state` | `DroneState` | Stan początkowy drona (pozycja, prędkość, orientacja). |
| `model` | `&dyn VehicleModel` | Model dynamiki pojazdu. |
| `config` | `&SimConfig` | Konfiguracja symulacji (krok czasowy, czas trwania). |
| `integrator` | `&dyn Integrator` | Metoda całkowania (np. `&RK4`). |
| `controller` | `impl FnMut(&DroneState, TimeStep) -> KnownActuatorInput` | Domknięcie kontrolera — na podstawie bieżącego stanu i kroku `dt` zwraca sygnał sterujący. Sygnatura jest identyczna z `Controller::update`, co ułatwia adaptację implementacji traita `Controller` do tego interfejsu. |

**Zwraca:** `Vec<SimFrame>` — wektor klatek od `t = 0` (stan początkowy) do `t ≈ duration`. Wektor zawiera `steps + 1` elementów (włącznie z klatką początkową).

**Przebieg jednej iteracji pętli:**

1. **Kontroler** — wywołanie domknięcia `controller(&state, dt)` → `KnownActuatorInput`.
2. **Aktuatory** — `model.step_actuators(&mut state, &input, dt)` aktualizuje wewnętrzny stan aktuatorów (np. dynamikę silników).
3. **Całkowanie** — `integrator.step(model, &state, &input, dt)` oblicza nowy stan drona.
4. **Zapis klatki** — nowy stan wraz z aktualnym czasem jest dodawany do wektora wynikowego.
