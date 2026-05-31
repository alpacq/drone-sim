# Narzędzia wiersza poleceń (CLI)

## Przegląd

Projekt `drone-sim` zawiera cztery programy wykonywalnych (crate'y binarne w `bin/`):

| Narzędzie | Opis |
|---|---|
| `sitl-test` | Uruchamia scenariusze SITL i raportuje wynik pass/fail dla każdego z nich. |
| `sitl-compare` | Porównuje kilka regulatorów lotu obok siebie na tym samym zestawie scenariuszy. |
| `monte-carlo` | Symulacja Monte Carlo — wielokrotne uruchomienie scenariusza z zaburzonymi warunkami początkowymi. |
| `telem-analyze` | Walidacja modelu fizycznego na podstawie rzeczywistej telemetrii DJI (pliki SRT). |

Wszystkie programy budowane są standardowo:

```sh path=null start=null
cargo build -p sitl-test -p sitl-compare -p monte-carlo -p telem-analyze
```

---

## sitl-test

Uruchamia zestaw scenariuszy SITL (Software-In-The-Loop) z wybranym regulatorem i sprawdza, czy metryki lotu spełniają zdefiniowane kryteria akceptacji.

### Flagi CLI

```text path=null start=null
sitl-test [OPTIONS]
```

| Flaga | Typ | Domyślnie | Opis |
|---|---|---|---|
| `--scenarios-dir <ŚCIEŻKA>` | `PathBuf` | `scenarios` | Katalog zawierający pliki scenariuszy TOML. |
| `--controller <RODZAJ>` | `ControllerKind` | `cascade` | Regulator dla scenariuszy quadrotora. Scenariusze F-16 **zawsze** używają wbudowanego LQR niezależnie od tej flagi. |
| `--config <ŚCIEŻKA>`, `-c` | `PathBuf` (opcjonalnie) | brak | Plik TOML z parametrami regulatora. Gdy podany, nadpisuje `--controller` — pole `type` w pliku decyduje o rodzaju regulatora. |
| `--plot` | `bool` | `false` | Generuje wykresy PNG odpowiedzi skokowej w katalogu `target/` dla każdego scenariusza. |

### Enum `ControllerKind`

Określa rodzaj regulatora dla scenariuszy quadrotora:

```rust path=null start=null
enum ControllerKind {
    /// Kaskadowy PID (pozycja → prędkość → orientacja). Domyślny.
    Cascade,
    /// Regulator liniowo-kwadratowy — stabilizuje wokół punktu pracy.
    Lqr,
    /// Regulator liniowo-kwadratowy z członem całkowym — śledzi zadany punkt.
    Lqi,
}
```

### Obsługa F-16

Scenariusze z `vehicle = "f16"` korzystają z dedykowanej fabryki `f16_lqr_factory`, która:

1. **Rozgrzewa silnik odrzutowy** — 1000 kroków po 0.01 s (10 s >> 5τ = 0.5 s), aby uniknąć dywergencji solvera CARE przy zerowym ciągu.
2. **Buduje stan trymowania** — lot poziomy na poziomie morza, V = 200 m/s, kąt natarcia α = 5°.
3. **Projektuje regulator LQR** z wagami Q (13 stanów) i R (4 aktuatory: throttle, aileron, elevator, rudder).

Flaga `--controller` jest ignorowana dla scenariuszy F-16.

### Przykłady użycia

Podstawowe uruchomienie ze wszystkimi scenariuszami i domyślnym regulatorem kaskadowym:

```sh path=null start=null
cargo run -p sitl-test
```

Z regulatorem LQR:

```sh path=null start=null
cargo run -p sitl-test -- --controller lqr
```

Z plikiem konfiguracyjnym regulatora:

```sh path=null start=null
cargo run -p sitl-test -- --config controllers/cascade.toml
```

Z generowaniem wykresów:

```sh path=null start=null
cargo run -p sitl-test -- --controller lqi --plot
```

### Format wyjścia

Dla każdego scenariusza program wypisuje wynik (PASS/FAIL) z wartościami metryk i ich progami. Na końcu wypisywane jest podsumowanie:

```text path=null start=null
═══════════════════════════════
  Results: 8 PASS, 1 FAIL
═══════════════════════════════
```

Dodatkowo generowany jest raport Markdown w `target/sitl_report_RRRR-MM-DD_GG-MM.md` zawierający tabelę ze wszystkimi scenariuszami, wynikami, wartościami metryk i odnośnikami do wykresów (jeśli `--plot`).

Program kończy się kodem `1` jeśli którykolwiek scenariusz nie przeszedł.

---

## sitl-compare

Porównuje kilka regulatorów lotu na tym samym zestawie scenariuszy, generując tabele metryk, pliki CSV i wykresy.

### Flagi CLI

```text path=null start=null
sitl-compare [OPTIONS]
```

| Flaga | Typ | Domyślnie | Opis |
|---|---|---|---|
| `--config <ŚCIEŻKA>`, `-c` | `PathBuf` (opcjonalnie) | brak | Plik TOML z listą regulatorów do porównania (format `CompareConfig`). Gdy pominięty, używany jest domyślny zestaw 4 regulatorów. |
| `--scenarios <ŚCIEŻKI>` | `Vec<PathBuf>` (opcjonalnie, oddzielone przecinkami) | `scenarios/step_response.toml`, `scenarios/disturbance_rejection.toml`, `scenarios/turbulence_comparison.toml` | Pliki scenariuszy TOML do uruchomienia. |

### Domyślny zestaw regulatorów

Gdy nie podano `--config`, porównywane są cztery regulatory:

1. **Cascade-PID** — kaskadowy PID z domyślnymi parametrami
2. **LQR-R=0.01** — LQR z domyślnymi wagami R (agresywny)
3. **LQR-R=1.0** — LQR z R = 1.0 na każdy silnik (łagodny)
4. **LQI** — LQI z domyślnymi parametrami

### Format konfiguracji TOML (`CompareConfig`)

Plik definiuje tablicę `[[controllers]]`, gdzie każdy wpis zawiera `name` i zagnieżdżoną tabelę `[controllers.config]`:

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

Struktura `NamedController`:

```rust path=null start=null
struct NamedController {
    /// Etykieta wyświetlana w tabeli porównawczej.
    name: String,
    /// Konfiguracja regulatora (ten sam format co pliki w controllers/).
    config: ControllerConfig,
}
```

### Wyjście

Dla każdego scenariusza program generuje:

- **Tabelę porównawczą** na stdout z kolumnami: Controller, RMS Z [m], OS [%] (overshoot), ST [s] (settling time), RT [s] (rise time), Energy
- **CSV z trajektoriami** — `target/{scenariusz}_trajectories.csv`
- **CSV z metrykami** — `target/{scenariusz}_metrics.csv`
- **Wykresy PNG** — `target/{scenariusz}_trajectories.png` i `target/{scenariusz}_metrics.png`
- **Raport Markdown** — `target/report_RRRR-MM-DD_GG-MM.md`

### Przykłady użycia

Porównanie domyślnych regulatorów na domyślnych scenariuszach:

```sh path=null start=null
cargo run -p sitl-compare
```

Z własną konfiguracją regulatorów:

```sh path=null start=null
cargo run -p sitl-compare -- --config controllers/compare.toml
```

Z wybranym scenariuszem:

```sh path=null start=null
cargo run -p sitl-compare -- --scenarios scenarios/step_response.toml,scenarios/hover_stability.toml
```

---

## monte-carlo

Uruchamia scenariusz SITL wielokrotnie z losowo zaburzonymi warunkami początkowymi (pozycja, prędkość) i agreguje statystyki metryk. Poszczególne przebiegi wykonywane są równolegle.

### Flagi CLI

```text path=null start=null
monte-carlo [OPTIONS] -s <ŚCIEŻKA>
```

| Flaga | Typ | Domyślnie | Opis |
|---|---|---|---|
| `--scenario <ŚCIEŻKA>`, `-s` | `PathBuf` | (wymagany) | Ścieżka do pliku scenariusza TOML. |
| `--runs <N>` | `usize` | `100` | Liczba niezależnych przebiegów symulacji. |
| `--pos-noise <σ>` | `f64` | `0.5` | Odchylenie standardowe szumu pozycji początkowej [m]. |
| `--vel-noise <σ>` | `f64` | `0.1` | Odchylenie standardowe szumu prędkości początkowej [m/s]. |
| `--seed <SEED>` | `u64` | `42` | Ziarno generatora liczb pseudolosowych (powtarzalność wyników). |
| `--controller <RODZAJ>` | `ControllerKind` | `cascade` | Rodzaj regulatora (`cascade`, `lqr`, `lqi`). |
| `--config <ŚCIEŻKA>`, `-c` | `PathBuf` (opcjonalnie) | brak | Plik TOML z parametrami regulatora. Gdy podany, nadpisuje `--controller`. |

### Działanie

1. Wczytuje scenariusz TOML i tworzy model `QuadrotorModel::mini3()`.
2. Dla każdego z N przebiegów dodaje szum gaussowski do pozycji i prędkości początkowej.
3. Uruchamia wszystkie przebiegi równolegle.
4. Agreguje statystyki (średnia, odchylenie standardowe, min, max) dla każdej metryki.

### Wyjście

- **Tabela statystyk** na stdout
- **Plik CSV** — `target/{scenariusz}_mc.csv`
- **Wykres PNG** — `target/{scenariusz}_mc.png`

### Przykłady użycia

Podstawowe uruchomienie (100 przebiegów, domyślny szum):

```sh path=null start=null
cargo run -p monte-carlo -- -s scenarios/step_response.toml
```

500 przebiegów z większym szumem i regulatorem LQI:

```sh path=null start=null
cargo run -p monte-carlo -- \
    -s scenarios/step_response.toml \
    --runs 500 \
    --pos-noise 1.0 \
    --vel-noise 0.3 \
    --controller lqi
```

Z plikiem konfiguracyjnym regulatora i ustalonym ziarnem:

```sh path=null start=null
cargo run -p monte-carlo -- \
    -s scenarios/disturbance_rejection.toml \
    -c controllers/lqr.toml \
    --seed 12345 \
    --runs 200
```

---

## telem-analyze

Walidacja modelu fizycznego drona na podstawie rzeczywistej telemetrii DJI. Program parsuje plik SRT z napisami, normalizuje punkty GPS do układu ENU, uruchamia symulację open-loop modelu Mini 3 i porównuje trajektorie.

### Flagi CLI

```text path=null start=null
telem-analyze [OPTIONS] [PLIK]
```

| Flaga | Typ | Domyślnie | Opis |
|---|---|---|---|
| `<file>` (argument pozycyjny) | `PathBuf` | `data/DJI_0001.srt` | Plik DJI `.srt` do analizy. |
| `--dt-s <DT>`, `-d` | `f64` (opcjonalnie) | średni interwał klatek SRT | Krok czasowy symulacji [s]. Domyślnie obliczany jako odwrotność częstotliwości klatek telemetrii. |
| `--threshold-m <PRÓG>` | `f64` | `VALID_POSITION_THRESHOLD_M` | Próg błędu pozycji [m] dla metryki „model ważny do czasu t". Punkt jest uznawany za poprawny, gdy `|pos_error| < threshold`. |
| `--save-csv`, `-o` | `bool` | `false` | Zapisuje tabelę porównawczą punkt-po-punkcie do pliku CSV obok pliku wejściowego. |
| `--plot` | `bool` | `false` | Generuje wykres walidacyjny PNG w katalogu `target/`. |

### Pipeline przetwarzania

1. **Parsowanie SRT** — `parse_file()` wyodrębnia klatki telemetryczne z pliku DJI SRT.
2. **Normalizacja GPS** — `normalize()` przelicza współrzędne GPS na lokalny układ ENU (East-North-Up), oblicza czas trwania lotu, maksymalną wysokość i prędkość.
3. **Symulacja open-loop** — model `QuadrotorModel::mini3()` jest uruchamiany z krokiem `dt` (domyślnie wyznaczonym z częstotliwości telemetrii, fallback 30 fps = 0.033 s).
4. **Wyrównanie i porównanie** — `validate_model()` porównuje trajektorie modelu i telemetrii, oblicza metryki błędu.
5. **Raport** — wyniki wypisywane na stdout z metrykami pozycji i prędkości.

### Format wyjścia

Na stdout wypisywane są:

- Liczba sparsowanych klatek SRT
- Czas trwania lotu i liczba punktów GPS
- Maksymalna wysokość [m] i prędkość [m/s, km/h]
- Metryki walidacyjne (z `report.print()`)

Opcjonalnie:
- **CSV** — `{plik_wejściowy}.validation.csv` (z flagą `--save-csv`)
- **PNG** — wykres w `target/` (z flagą `--plot`)

### Przykłady użycia

Analiza domyślnego pliku telemetrycznego:

```sh path=null start=null
cargo run -p telem-analyze
```

Analiza konkretnego pliku z zapisem CSV i wykresem:

```sh path=null start=null
cargo run -p telem-analyze -- data/DJI_0042.srt --save-csv --plot
```

Z niestandardowym krokiem czasowym i progiem:

```sh path=null start=null
cargo run -p telem-analyze -- data/DJI_0042.srt -d 0.02 --threshold-m 5.0
```

---

## Przykłady konfiguracji TOML

### Regulator kaskadowy PID (`cascade.toml`)

Trójpoziomowy kaskadowy regulator PID: pozycja → prędkość → orientacja → komendy silników.

```toml path=null start=null
type = "cascade"

# Maksymalny kąt przechylenia dla sterowania XY [deg].
max_tilt_deg = 8.6

# Pętla prędkości pionowej: błąd vz → delta throttle
[vel_z]
kp             = 0.3
ki             = 0.1
kd             = 0.0
integral_limit = 0.45
output_limit   = 0.45

# Pętle prędkości poziomej (wspólna konfiguracja dla X i Y): błąd vx/vy → zadany pitch/roll
[vel_xy]
kp             = 0.4
ki             = 0.05
kd             = 0.0
integral_limit = 0.5
output_limit   = 0.35

# Pętle orientacji (wspólna konfiguracja dla roll i pitch): błąd kąta → delta komend silników
[att]
kp             = 4.0
ki             = 0.0
kd             = 0.2
integral_limit = 1.0
output_limit   = 1.0

# Pętla yaw
[att_yaw]
kp             = 2.0
ki             = 0.1
kd             = 0.0
integral_limit = 0.5
output_limit   = 0.5
```

Parametry `PidConfig` dla każdej pętli:

- `kp` — wzmocnienie proporcjonalne
- `ki` — wzmocnienie całkowe
- `kd` — wzmocnienie różniczkowe
- `integral_limit` — ograniczenie anty-windup na akumulatorze całki
- `output_limit` — ograniczenie na wyjściu pętli

### Regulator LQR (`lqr.toml`)

Regulator liniowo-kwadratowy. Solver CARE uruchamiany raz na scenariusz. Stabilizuje punkt trymowania, **nie** śledzi zadanych punktów.

```toml path=null start=null
type = "lqr"

# Wysokość punktu trymowania do linearyzacji [m].
trim_z_m = 5.0

# Wektor wag Q — 13 elementów (wektor stanu quadrotora):
#   [x, y, z, vx, vy, vz, ωx, ωy, ωz, qi, qj, qk, qw]
# Wyższe wagi z/vz poprawiają śledzenie wysokości;
# wyższe wagi kwaternionów utrzymują drona w poziomie.
q_weights = [
  1.0,  1.0,  50.0,
  0.5,  0.5,   5.0,
  2.0,  2.0,   2.0,
  20.0, 20.0, 20.0, 20.0,
]

# Wektor wag R — 4 elementy (jeden na silnik).
# Większe wartości → łagodniejsze, mniej agresywne sterowanie.
r_weights = [0.01, 0.01, 0.01, 0.01]
```

### Regulator LQI (`lqi.toml`)

Rozszerzenie LQR o cztery stany całkowe `[ξ_x, ξ_y, ξ_z, ξ_ψ]`, eliminujące błąd ustalony pozycji/yaw przy stałych zaburzeniach (wiatr, opór, spadek napięcia baterii).

```toml path=null start=null
type = "lqi"

trim_z_m = 5.0

# Wektor wag Q — 17 elementów:
#   13 stanów obiektu (jak LQR) + 4 stany całkowe [ξ_x, ξ_y, ξ_z, ξ_ψ]
q_weights = [
  # 13 wag obiektu
  1.0,  1.0,  50.0,
  0.5,  0.5,   5.0,
  2.0,  2.0,   2.0,
  20.0, 20.0, 20.0, 20.0,
  # 4 wagi całkowe
  5.0,  5.0,  30.0,  2.0,
]

r_weights = [0.01, 0.01, 0.01, 0.01]

# Ograniczenie anty-windup dla każdej całki [m·s, m·s, m·s, rad·s].
# Opcjonalne — domyślnie [30, 30, 30, 2π].
# xi_limits = [30.0, 30.0, 30.0, 6.2832]
```

### Plik scenariusza TOML

Każdy scenariusz definiuje warunki początkowe, cel, czas trwania i kryteria akceptacji:

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

Scenariusze F-16 dodatkowo definiują `vehicle = "f16"` i mogą podawać orientację początkową w stopniach (`attitude_deg = [roll, pitch, yaw]`).
