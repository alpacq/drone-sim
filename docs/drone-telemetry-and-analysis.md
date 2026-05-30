# drone-telemetry & drone-analysis — dokumentacja referencyjna

---

## drone-telemetry

### Przegląd

Crate `drone-telemetry` parsuje pliki napisów SRT generowane przez drony DJI i przekształca je w ustrukturyzowane dane telemetryczne. Pipeline składa się z trzech etapów:

1. **Parsowanie** (`parser`) — odczyt pliku SRT, podział na bloki, ekstrakcja pól.
2. **Ramki** (`frame`) — reprezentacja pojedynczej ramki telemetrycznej ze wszystkimi polami.
3. **Normalizacja** (`normalize`) — konwersja GPS → układ ENU, obliczanie prędkości, budowa trajektorii lotu.

Crate stosuje **tolerancyjne parsowanie**: nieprawidłowe bloki SRT są pomijane z ostrzeżeniem, a przetwarzanie kontynuuje się dla pozostałych.

---

### Moduł `frame`

#### Struktura `TelemetryFrame`

Reprezentuje pojedynczą ramkę telemetryczną wyekstrahowaną z bloku SRT.

```rust
#[derive(Debug, Clone)]
pub struct TelemetryFrame {
    pub index: u32,
    pub timestamp: Option<DateTime<Utc>>,
    pub duration_ms: u32,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub rel_alt: Option<f32>,
    pub abs_alt: Option<f32>,
    pub gimbal_yaw: Option<f32>,
    pub gimbal_pitch: Option<f32>,
    pub gimbal_roll: Option<f32>,
    pub iso: Option<u32>,
    pub shutter: Option<String>,
    pub fnum: Option<u32>,
    pub color_temp: Option<u32>,
}
```

**Pola:**

- `index` (`u32`) — numer kolejny bloku SRT (1-indeksowany).
- `timestamp` (`Option<DateTime<Utc>>`) — znacznik czasu z bloku SRT. `None` jeśli nie udało się sparsować.
- `duration_ms` (`u32`) — czas trwania ramki w milisekundach (pole `DiffTime`). Domyślnie 33 ms jeśli brak w danych.
- `latitude` (`Option<f64>`) — szerokość geograficzna w stopniach.
- `longitude` (`Option<f64>`) — długość geograficzna w stopniach.
- `rel_alt` (`Option<f32>`) — wysokość względna (nad punktem startu) w metrach.
- `abs_alt` (`Option<f32>`) — wysokość bezwzględna (n.p.m.) w metrach.
- `gimbal_yaw` (`Option<f32>`) — odchylenie gimbala (yaw) w stopniach.
- `gimbal_pitch` (`Option<f32>`) — pochylenie gimbala (pitch) w stopniach.
- `gimbal_roll` (`Option<f32>`) — przechylenie gimbala (roll) w stopniach.
- `iso` (`Option<u32>`) — czułość ISO kamery.
- `shutter` (`Option<String>`) — czas otwarcia migawki (np. `"1/1000"`).
- `fnum` (`Option<u32>`) — liczba przysłony (wartość × 100, np. 280 = f/2.8).
- `color_temp` (`Option<u32>`) — temperatura barwowa w kelwinach.

#### Metody

##### `dt_seconds`

```rust
pub fn dt_seconds(&self) -> f64
```

Zwraca czas trwania ramki w sekundach (`duration_ms / 1000.0`). Używane przy kumulowaniu czasu trajektorii.

##### `has_gps`

```rust
pub fn has_gps(&self) -> bool
```

Zwraca `true` jeśli ramka zawiera zarówno `latitude` jak i `longitude`. Służy do filtrowania ramek nadających się do normalizacji trajektorii.

---

### Moduł `parser`

#### Funkcja `parse_file`

```rust
pub fn parse_file(path: &Path) -> Result<Vec<TelemetryFrame>, TelemetryError>
```

Wczytuje plik SRT z dysku i parsuje jego zawartość.

**Parametry:**
- `path` (`&Path`) — ścieżka do pliku SRT.

**Zwraca:** `Result<Vec<TelemetryFrame>, TelemetryError>` — wektor sparsowanych ramek lub błąd `TelemetryError::Io` / `TelemetryError::Empty`.

**Kiedy używać:** Gdy dane telemetryczne znajdują się w pliku na dysku.

#### Funkcja `parse_str`

```rust
pub fn parse_str(content: &str) -> Result<Vec<TelemetryFrame>, TelemetryError>
```

Parsuje zawartość SRT podaną jako łańcuch znaków.

**Parametry:**
- `content` (`&str`) — surowa zawartość pliku SRT.

**Zwraca:** `Result<Vec<TelemetryFrame>, TelemetryError>` — wektor ramek lub `TelemetryError::Empty` jeśli żaden blok nie był prawidłowy.

**Kiedy używać:** Gdy zawartość SRT jest już w pamięci (np. pobrana z sieci, wklejona).

**Tolerancyjne parsowanie:** Nieprawidłowe bloki SRT są pomijane z komunikatem ostrzegawczym na `stderr`. Błąd zwracany jest tylko jeśli *żaden* blok nie dał się sparsować.

**Format bloku SRT:**

Każdy blok SRT ma strukturę:

```
<numer>
<zakres czasu>
<zawartość: DiffTime, timestamp, pary klucz-wartość w nawiasach [ ]>
```

Pary klucz-wartość parsowane z nawiasów kwadratowych: `iso`, `shutter`, `fnum`, `ct` (temperatura barwowa), `latitude`, `longitude`, `rel_alt`, `abs_alt`, `gb_yaw`, `gb_pitch`, `gb_roll`. Tagi `<font>` są automatycznie usuwane.

---

### Moduł `normalize`

#### Struktura `TrajectoryPoint`

Pojedynczy punkt znormalizowanej trajektorii w układzie ENU.

```rust
#[derive(Debug, Clone)]
pub struct TrajectoryPoint {
    pub time: f64,
    pub position: Vector3<f64>,
    pub velocity: Option<Vector3<f64>>,
    pub frame_idx: u32,
}
```

**Pola:**
- `time` (`f64`) — czas od początku lotu w sekundach (kumulowany z `duration_ms` ramek).
- `position` (`Vector3<f64>`) — pozycja w układzie ENU (East-North-Up) w metrach względem punktu początkowego.
- `velocity` (`Option<Vector3<f64>>`) — prędkość w m/s obliczona różnicami centralnymi. `None` jeśli nie da się obliczyć.
- `frame_idx` (`u32`) — indeks oryginalnej ramki SRT.

#### Struktura `FlightTrajectory`

Pełna znormalizowana trajektoria lotu.

```rust
#[derive(Debug, Clone)]
pub struct FlightTrajectory {
    pub points: Vec<TrajectoryPoint>,
    pub origin: GpsOrigin,
    pub duration_s: f64,
}
```

**Pola:**
- `points` (`Vec<TrajectoryPoint>`) — uporządkowane chronologicznie punkty trajektorii.
- `origin` (`GpsOrigin`) — punkt odniesienia GPS (pierwsza ramka z danymi GPS).
- `duration_s` (`f64`) — całkowity czas trwania trajektorii w sekundach.

**Metody:**
- `len() -> usize` — liczba punktów trajektorii.
- `is_empty() -> bool` — czy trajektoria jest pusta.

#### Struktura `GpsOrigin`

Punkt odniesienia (origin) dla transformacji GPS → ENU. Odpowiada pierwszej ramce z danymi GPS.

```rust
#[derive(Debug, Clone, Copy)]
pub struct GpsOrigin {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}
```

**Pola:**
- `latitude` (`f64`) — szerokość geograficzna w stopniach.
- `longitude` (`f64`) — długość geograficzna w stopniach.
- `altitude` (`f64`) — wysokość względna w metrach.

#### Funkcja `normalize`

```rust
pub fn normalize(frames: &[TelemetryFrame]) -> Result<FlightTrajectory, TelemetryError>
```

Przekształca surowe ramki telemetryczne w znormalizowaną trajektorię lotu.

**Parametry:**
- `frames` (`&[TelemetryFrame]`) — sparsowane ramki z modułu `parser`.

**Zwraca:** `Result<FlightTrajectory, TelemetryError>` — trajektoria lub błąd.

**Działanie:**
1. Filtruje ramki posiadające dane GPS (`has_gps()`) i wysokość względną (`rel_alt`).
2. Wyznacza punkt odniesienia `GpsOrigin` z pierwszej ramki GPS.
3. Konwertuje współrzędne GPS każdej ramki do układu ENU za pomocą `gps_to_enu()`.
4. Oblicza prędkość **różnicami centralnymi**: dla punktu wewnętrznego `v[i] = (pos[i+1] - pos[i-1]) / (t[i+1] - t[i-1])`, dla krańcowych — różnicą prostą/wsteczną.
5. Czas kumulowany jest z pola `duration_ms` kolejnych ramek.

**Błędy:**
- `TelemetryError::NoGpsFrames` — brak ramek z danymi GPS.
- `TelemetryError::NotEnoughGpsFrames` — mniej niż 3 ramki GPS (minimum do obliczenia prędkości różnicami centralnymi).

#### Funkcja `gps_to_enu`

```rust
pub fn gps_to_enu(lat: f64, lon: f64, alt: f64, origin: &GpsOrigin) -> Vector3<f64>
```

Konwertuje współrzędne GPS na pozycję w lokalnym układzie ENU (East-North-Up).

**Parametry:**
- `lat` (`f64`) — szerokość geograficzna w stopniach.
- `lon` (`f64`) — długość geograficzna w stopniach.
- `alt` (`f64`) — wysokość w metrach.
- `origin` (`&GpsOrigin`) — punkt odniesienia.

**Zwraca:** `Vector3<f64>` — pozycja `(x, y, z)` w metrach, gdzie:
- `x` — przesunięcie na wschód (East),
- `y` — przesunięcie na północ (North),
- `z` — przesunięcie w górę (Up, = różnica wysokości).

Przybliżenie płaskie ze stałą `METERS_PER_DEGREE = 111_320.0`, z korektą kosinusową dla długości geograficznej.

#### Funkcja `enu_to_gps`

```rust
pub fn enu_to_gps(enu: &Vector3<f64>, origin: &GpsOrigin) -> (f64, f64, f64)
```

Konwersja odwrotna: z układu ENU do współrzędnych GPS.

**Parametry:**
- `enu` (`&Vector3<f64>`) — pozycja w układzie ENU w metrach.
- `origin` (`&GpsOrigin`) — punkt odniesienia.

**Zwraca:** `(f64, f64, f64)` — krotka `(latitude, longitude, altitude)` w stopniach/metrach.

Transformacja odwrotna do `gps_to_enu()`. Obie funkcje tworzą parę bijektywną (round-trip).

---

### Moduł `error`

#### Enum `TelemetryError`

Błędy generowane przez pipeline parsowania i normalizacji telemetrii.

```rust
#[derive(Debug, Error)]
pub enum TelemetryError {
    Io { path: String, source: std::io::Error },
    Empty,
    NoGpsFrames,
    NotEnoughGpsFrames { found: usize },
}
```

**Warianty:**

- `Io { path: String, source: std::io::Error }` — błąd odczytu pliku z dysku. Pole `path` zawiera ścieżkę pliku, `source` — oryginalny błąd I/O.
- `Empty` — zawartość SRT nie zawierała żadnych prawidłowych bloków telemetrycznych. Plik może być pusty lub mieć nierozpoznawalny format.
- `NoGpsFrames` — żadna sparsowana ramka nie zawiera danych GPS. Sprawdź, czy funkcja Video Captions była włączona podczas lotu.
- `NotEnoughGpsFrames { found: usize }` — znaleziono mniej niż 3 ramki GPS (pole `found` zawiera faktyczną liczbę). Minimum 3 ramki wymagane do obliczenia prędkości różnicami centralnymi.

---

## drone-analysis

### Przegląd

Crate `drone-analysis` służy do walidacji modelu fizycznego drona poprzez porównanie wyników symulacji z rzeczywistą telemetrią. Pipeline walidacji:

1. **Walidacja** (`validate`) — uruchamia symulację open-loop z parametrami równowagi modelu i konfiguracją z telemetrii.
2. **Wyrównanie** (`align`) — interpoluje wyniki symulacji do chwil czasowych telemetrii i oblicza błędy.
3. **Raport** (`report`) — agreguje metryki błędów i generuje czytelny raport z oceną jakości modelu.

---

### Moduł `align`

#### Struktura `AlignedPoint`

Pojedynczy punkt porównania: symulacja vs. telemetria w tej samej chwili czasowej.

```rust
#[derive(Debug, Clone)]
pub struct AlignedPoint {
    pub time: f64,
    pub sim_pos: Vector3<f64>,
    pub sim_vel: Vector3<f64>,
    pub telem_pos: Vector3<f64>,
    pub telem_vel: Vector3<f64>,
    pub pos_error: f64,
    pub vel_error: f64,
}
```

**Pola:**
- `time` (`f64`) — chwila czasowa w sekundach.
- `sim_pos` (`Vector3<f64>`) — pozycja z symulacji (interpolowana) w metrach.
- `sim_vel` (`Vector3<f64>`) — prędkość z symulacji (interpolowana) w m/s.
- `telem_pos` (`Vector3<f64>`) — pozycja z telemetrii w metrach.
- `telem_vel` (`Vector3<f64>`) — prędkość z telemetrii w m/s (wektor zerowy jeśli brak danych).
- `pos_error` (`f64`) — norma euklidesowa błędu pozycji `‖sim_pos - telem_pos‖` w metrach.
- `vel_error` (`f64`) — norma euklidesowa błędu prędkości `‖sim_vel - telem_vel‖` w m/s. Wynosi 0.0 jeśli telemetria nie zawiera prędkości.

#### Struktura `AlignedTrajectory`

Wyrównana trajektoria — sekwencja punktów porównania z metrykami zagregowanymi.

```rust
pub struct AlignedTrajectory {
    pub points: Vec<AlignedPoint>,
    pub duration_s: f64,
}
```

**Pola:**
- `points` (`Vec<AlignedPoint>`) — chronologicznie uporządkowane punkty porównania.
- `duration_s` (`f64`) — czas trwania wyrównanej trajektorii w sekundach.

**Metody:**

##### `position_rms`

```rust
pub fn position_rms(&self) -> f64
```

Zwraca błąd RMS (Root Mean Square) pozycji w metrach. Obliczenie: `√(Σ pos_error² / n)`. Zwraca 0.0 dla pustej trajektorii.

##### `position_max`

```rust
pub fn position_max(&self) -> f64
```

Zwraca maksymalny błąd pozycji w metrach spośród wszystkich punktów.

##### `velocity_rms`

```rust
pub fn velocity_rms(&self) -> f64
```

Zwraca błąd RMS prędkości w m/s. Obliczenie analogiczne do `position_rms`. Zwraca 0.0 dla pustej trajektorii.

##### `to_csv`

```rust
pub fn to_csv(&self) -> String
```

Eksportuje wyrównaną trajektorię do formatu CSV. Kolumny: `time`, `sim_x`, `sim_y`, `sim_z`, `telem_x`, `telem_y`, `telem_z`, `pos_error`, `vel_error`.

#### Funkcja `align`

```rust
pub fn align(sim_frames: &[SimFrame], telemetry: &FlightTrajectory) -> AlignedTrajectory
```

Wyrównuje trajektorię symulacji z trajektorią telemetryczną.

**Parametry:**
- `sim_frames` (`&[SimFrame]`) — wyniki symulacji (z `drone_sim::runner`).
- `telemetry` (`&FlightTrajectory`) — znormalizowana trajektoria telemetryczna.

**Zwraca:** `AlignedTrajectory` — wyrównana trajektoria z obliczonymi błędami.

**Działanie:** Dla każdego punktu telemetrycznego interpoluje liniowo pozycję i prędkość symulacji do odpowiedniej chwili czasowej. Punkty telemetryczne poza zakresem czasowym symulacji używają wartości z pierwszej/ostatniej klatki symulacji (ekstrapolacja stała).

---

### Moduł `validate`

#### Struktura `ValidateConfig`

Konfiguracja walidacji modelu.

```rust
#[derive(Debug, Clone)]
pub struct ValidateConfig {
    pub dt: TimeStep,
    pub valid_position_threshold_m: f64,
}
```

**Pola:**
- `dt` (`TimeStep`) — krok czasowy integracji symulacji. Domyślnie: 0.02 s (50 Hz).
- `valid_position_threshold_m` (`f64`) — próg błędu pozycji w metrach do wyznaczenia czasu `valid_until_s`. Domyślnie: `VALID_POSITION_THRESHOLD_M` (2.0 m).

Implementuje `Default` z powyższymi wartościami domyślnymi.

#### Funkcja `validate_model`

```rust
pub fn validate_model(
    model: &dyn VehicleModel,
    telemetry: &FlightTrajectory,
    config: ValidateConfig,
    source_file: String,
) -> Result<ValidationReport, AnalysisError>
```

Uruchamia symulację open-loop i porównuje z telemetrią.

**Parametry:**
- `model` (`&dyn VehicleModel`) — model fizyczny drona do walidacji.
- `telemetry` (`&FlightTrajectory`) — znormalizowana trajektoria referencyjna.
- `config` (`ValidateConfig`) — konfiguracja walidacji.
- `source_file` (`String`) — nazwa pliku źródłowego (do raportu).

**Zwraca:** `Result<ValidationReport, AnalysisError>` — raport walidacji lub `AnalysisError::EmptyTrajectory`.

**Działanie:**
1. Ustawia stan początkowy symulacji na podstawie pierwszego punktu telemetrii (pozycja, prędkość; orientacja = tożsamość, prędkość kątowa = 0).
2. Uruchamia symulację z integratorem RK4 i **stałym wejściem równowagowym** (`model.equilibrium_input()`). Symulacja jest celowo open-loop — testowany jest model fizyczny, nie kontroler.
3. Wyrównuje trajektorie za pomocą `align()`.
4. Generuje `ValidationReport`.

---

### Moduł `report`

#### Stała `VALID_POSITION_THRESHOLD_M`

```rust
pub const VALID_POSITION_THRESHOLD_M: f64 = 2.0;
```

Domyślny próg błędu pozycji (2.0 m) używany do wyznaczenia `valid_until_s`. Wyeksponowany jako stała publiczna, aby `ValidateConfig` i kod wywołujący mógł się do niej odwoływać bez magicznej liczby.

#### Struktura `ValidationReport`

Raport walidacji modelu z zagregowanymi metrykami.

```rust
pub struct ValidationReport {
    pub source_file: String,
    pub flight_duration_s: f64,
    pub n_points: usize,
    pub position_rms_m: f64,
    pub position_max_m: f64,
    pub velocity_rms_ms: f64,
    pub valid_until_s: f64,
    pub trajectory: AlignedTrajectory,
}
```

**Pola:**
- `source_file` (`String`) — nazwa pliku źródłowego telemetrii.
- `flight_duration_s` (`f64`) — czas trwania lotu w sekundach.
- `n_points` (`usize`) — liczba punktów porównania.
- `position_rms_m` (`f64`) — RMS błędu pozycji w metrach.
- `position_max_m` (`f64`) — maksymalny błąd pozycji w metrach.
- `velocity_rms_ms` (`f64`) — RMS błędu prędkości w m/s.
- `valid_until_s` (`f64`) — czas (w sekundach od startu), do którego błąd pozycji nie przekracza progu. Wyznaczany jako czas ostatniego kolejnego punktu z `pos_error < valid_threshold_m` (licząc od początku).
- `trajectory` (`AlignedTrajectory`) — pełna wyrównana trajektoria do dalszej analizy lub eksportu.

**Metody:**

##### `from_aligned`

```rust
pub fn from_aligned(
    aligned: AlignedTrajectory,
    source_file: String,
    valid_threshold_m: f64,
) -> Self
```

Konstruktor tworzący raport z wyrównanej trajektorii.

**Parametry:**
- `aligned` (`AlignedTrajectory`) — wyrównana trajektoria.
- `source_file` (`String`) — nazwa pliku źródłowego.
- `valid_threshold_m` (`f64`) — próg błędu pozycji do wyznaczenia `valid_until_s`.

##### `print`

```rust
pub fn print(&self)
```

Wypisuje sformatowany raport na `stdout` z ramką ASCII. Zawiera ocenę jakości modelu:
- **Excellent** — RMS < 0.5 m
- **Good** — RMS < 2.0 m
- **Approximate** — RMS < 5.0 m
- **Poor** — RMS ≥ 5.0 m (zalecana weryfikacja parametrów modelu)

##### `to_csv`

```rust
pub fn to_csv(&self) -> String
```

Deleguje do `AlignedTrajectory::to_csv()`. Zwraca dane trajektorii w formacie CSV.

---

### Moduł `error`

#### Enum `AnalysisError`

Błędy zwracane przez pipeline walidacji modelu.

```rust
#[derive(Debug, Error)]
pub enum AnalysisError {
    EmptyTrajectory,
}
```

**Warianty:**
- `EmptyTrajectory` — trajektoria telemetryczna przekazana do `validate_model()` nie zawiera żadnych punktów. Walidacja wymaga co najmniej jednego punktu do porównania.
