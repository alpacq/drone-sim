pub trait AtmosphereModel: Send + Sync {
    fn density(&self, altitude_m: f64) -> f64;

    fn temperature(&self, altitude_m: f64) -> f64;

    fn speed_of_sound(&self, altitude_m: f64) -> f64;

    fn dynamic_pressure(&self, altitude_m: f64, speed_ms: f64) -> f64 {
        0.5 * self.density(altitude_m) * speed_ms * speed_ms
    }

    fn mach(&self, altitude_m: f64, speed_ms: f64) -> f64 {
        let a = self.speed_of_sound(altitude_m);
        if a > 0.0 { speed_ms / a } else { 0.0 }
    }

    /// Clone into a heap-allocated trait object.
    ///
    /// Required so that vehicle models storing `Box<dyn AtmosphereModel>` can
    /// be cloned (e.g. when a controller factory needs to own its own model
    /// copy for repeated linearisation).
    fn clone_box(&self) -> Box<dyn AtmosphereModel>;
}

mod isa_constants {
    /// Temperature at sea level [K] (= 15°C)
    pub const T0: f64 = 288.15;
    /// Pressure at sea level [Pa]
    pub const P0: f64 = 101_325.0;
    /// Temperature gradient in the troposphere [K/m]
    pub const L: f64 = 0.0065;
    /// Gas constant for air [J/(kg·K)]
    pub const R: f64 = 287.05;
    /// Gravitational acceleration [m/s²]
    pub const G: f64 = 9.80665;
    /// Isentropic exponent for air [-]
    pub const GAMMA: f64 = 1.4;
    /// Altitude of the tropopause [m] — above this, T = const
    pub const TROPOPAUSE_M: f64 = 11_000.0;
    /// Temperature at the tropopause [K]
    pub const T_TROPOPAUSE: f64 = 216.65;
    /// Pressure exponent: g/(R·L)
    pub const PRESSURE_EXPONENT: f64 = G / (R * L); // ≈ 5.2561
}

#[derive(Debug, Clone, Copy)]
pub struct Isa;

impl AtmosphereModel for Isa {
    fn clone_box(&self) -> Box<dyn AtmosphereModel> {
        Box::new(*self)
    }

    fn temperature(&self, altitude_m: f64) -> f64 {
        use isa_constants::*;
        let h = altitude_m.max(0.0);
        if h <= TROPOPAUSE_M {
            T0 - L * h
        } else {
            T_TROPOPAUSE
        }
    }

    fn density(&self, altitude_m: f64) -> f64 {
        use isa_constants::*;
        let h = altitude_m.max(0.0);
        let t = self.temperature(h);

        let p = if h <= TROPOPAUSE_M {
            P0 * (t / T0).powf(PRESSURE_EXPONENT)
        } else {
            let p_tropo = P0 * (T_TROPOPAUSE / T0).powf(PRESSURE_EXPONENT);
            let h_above = h - TROPOPAUSE_M;
            p_tropo * (-G * h_above / (R * T_TROPOPAUSE)).exp()
        };

        p / (R * t)
    }

    fn speed_of_sound(&self, altitude_m: f64) -> f64 {
        use isa_constants::*;
        let t = self.temperature(altitude_m);
        (GAMMA * R * t).sqrt()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ConstantDensity {
    pub density: f64,
    pub speed_of_sound: f64,
}

impl ConstantDensity {
    pub fn sea_level() -> Self {
        Self {
            density: 1.225,
            speed_of_sound: 340.29,
        }
    }
}

impl AtmosphereModel for ConstantDensity {
    fn clone_box(&self) -> Box<dyn AtmosphereModel> {
        Box::new(*self)
    }

    fn temperature(&self, _: f64) -> f64 {
        288.15
    }

    fn density(&self, _: f64) -> f64 {
        self.density
    }

    fn speed_of_sound(&self, _: f64) -> f64 {
        self.speed_of_sound
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn isa_sea_level() {
        let isa = Isa;
        assert!((isa.temperature(0.0) - 288.15).abs() < 0.01);
        assert!((isa.density(0.0) - 1.225).abs() < 0.001);
        assert!((isa.speed_of_sound(0.0) - 340.29).abs() < 0.1);
    }

    #[test]
    fn isa_5000m() {
        let isa = Isa;
        // T = 288.15 - 0.0065·5000 = 255.65 K
        assert!((isa.temperature(5000.0) - 255.65).abs() < 0.01);
        // ρ ≈ 0.7361 kg/m³
        assert!((isa.density(5000.0) - 0.7361).abs() < 0.001);
    }

    #[test]
    fn isa_11000m_tropopause() {
        let isa = Isa;
        // Exactly at the tropopause
        assert!((isa.temperature(11_000.0) - 216.65).abs() < 0.01);
    }

    #[test]
    fn isa_above_tropopause_temperature_constant() {
        let isa = Isa;
        // In the stratosphere, temperature remains constant
        assert!((isa.temperature(15_000.0) - 216.65).abs() < 0.01);
        assert!((isa.temperature(20_000.0) - 216.65).abs() < 0.01);
    }

    #[test]
    fn isa_density_decreases_with_altitude() {
        let isa = Isa;
        assert!(isa.density(1000.0) < isa.density(0.0));
        assert!(isa.density(5000.0) < isa.density(1000.0));
        assert!(isa.density(10000.0) < isa.density(5000.0));
    }

    #[test]
    fn mach_at_5000m_subsonic_f16() {
        let isa = Isa;
        // F-16 cruise: V ≈ 192 m/s na 5000m → Mach ≈ 0.6
        let v = 192.3;
        let m = isa.mach(5000.0, v);
        assert!((m - 0.6).abs() < 0.01, "Mach = {:.3}", m);
    }

    #[test]
    fn dynamic_pressure_increases_with_velocity() {
        let atmo = ConstantDensity::sea_level();
        let q1 = atmo.dynamic_pressure(0.0, 50.0);
        let q2 = atmo.dynamic_pressure(0.0, 100.0);
        // q ~ V² → double velocity = four times the pressure
        assert!((q2 / q1 - 4.0).abs() < 0.001);
    }

    #[test]
    fn constant_density_deterministic() {
        let atmo = ConstantDensity::sea_level();
        // Independent of altitude
        assert_eq!(atmo.density(0.0), atmo.density(5000.0));
        assert_eq!(atmo.density(0.0), atmo.density(10000.0));
    }
}
