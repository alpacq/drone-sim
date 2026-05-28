use crate::math::atmosphere::AtmosphereModel;
use crate::time::TimeStep;

#[derive(Debug, Clone)]
pub struct JetEngineParams {
    /// Max thrust at sea level, Mach=0 [N]
    pub thrust_sl_max: f64,
    /// Idle thrust at sea level [N] (min flow)
    pub thrust_sl_idle: f64,
    /// Time constant of the engine [s] — response time to flow change
    pub time_constant_s: f64,
    /// Altitude scaling exponent (Mach = 0)
    /// thrust(h) ≈ thrust_sl · (ρ(h)/ρ_sl)^altitude_exp
    pub altitude_exp: f64,
}

impl JetEngineParams {
    pub fn f110_dry() -> Self {
        Self {
            thrust_sl_max: 76_300.0, // N (~17,155 lbf)
            thrust_sl_idle: 4_500.0, // N (~1,000 lbf)
            time_constant_s: 0.5,    // s
            altitude_exp: 0.9,
        }
    }
}

/// Jet engine state
#[derive(Debug, Clone)]
pub struct JetEngine {
    pub params: JetEngineParams,
    /// Filtered throttle position [0, 1] — exposed so F16Model can mirror it
    /// into `DroneState::ActuatorState::FixedWingEngine`.
    pub current_throttle: f64,
    current_thrust_n: f64,
}

impl JetEngine {
    pub fn new(params: JetEngineParams) -> Self {
        Self {
            params,
            current_throttle: 0.0,
            current_thrust_n: 0.0,
        }
    }

    pub fn f110_dry() -> Self {
        Self::new(JetEngineParams::f110_dry())
    }

    pub fn thrust(&self) -> f64 {
        self.current_thrust_n
    }

    pub fn step(
        &mut self,
        throttle_cmd: f64,
        altitude_m: f64,
        mach: f64,
        atmosphere: &dyn AtmosphereModel,
        dt: TimeStep,
    ) {
        let cmd = throttle_cmd.clamp(0.0, 1.0);
        let alpha = (-dt.seconds() / self.params.time_constant_s).exp();
        self.current_throttle = alpha * self.current_throttle + (1.0 - alpha) * cmd;

        let thrust_max = self.thrust_at_conditions(altitude_m, mach, atmosphere);
        let thrust_idle = self.params.thrust_sl_idle
            * (atmosphere.density(altitude_m) / 1.225).powf(self.params.altitude_exp);
        self.current_thrust_n = thrust_idle + self.current_throttle * (thrust_max - thrust_idle);
    }

    fn thrust_at_conditions(
        &self,
        altitude_m: f64,
        mach: f64,
        atmosphere: &dyn AtmosphereModel,
    ) -> f64 {
        let p = &self.params;
        let rho_ratio = atmosphere.density(altitude_m) / 1.225;

        let thrust_altitude = p.thrust_sl_max * rho_ratio.powf(p.altitude_exp);
        let mach_factor = (1.0 - 0.096 * mach * mach).max(0.5);

        thrust_altitude * mach_factor
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::atmosphere::{ConstantDensity, Isa};

    #[test]
    fn engine_goes_to_command() {
        let mut engine = JetEngine::f110_dry();
        let atmo = ConstantDensity::sea_level();
        let dt = TimeStep::constant(0.01);

        // After long time engine should reach 100% throttle
        for _ in 0..1000 {
            engine.step(1.0, 0.0, 0.0, &atmo, dt);
        }

        assert!(
            (engine.current_throttle - 1.0).abs() < 0.001,
            "Throttle: {:.4}",
            engine.current_throttle
        );
    }

    #[test]
    fn thrust_lowers_with_altitude() {
        let mut e_sl = JetEngine::f110_dry();
        let mut e_5km = JetEngine::f110_dry();
        let isa = Isa;
        let dt = TimeStep::constant(0.01);

        // Stabilize on 100% throttle
        for _ in 0..500 {
            e_sl.step(1.0, 0.0, 0.3, &isa, dt);
            e_5km.step(1.0, 5000.0, 0.3, &isa, dt);
        }

        assert!(
            e_sl.thrust() > e_5km.thrust(),
            "Thrust at MSL ({:.0} N) should be > 5000m ({:.0} N)",
            e_sl.thrust(),
            e_5km.thrust()
        );
    }

    #[test]
    fn engine_doesn_not_respond_instantly() {
        let mut engine = JetEngine::f110_dry();
        let atmo = ConstantDensity::sea_level();
        let dt = TimeStep::constant(0.01);

        // After one step (10ms << τ=500ms) throttle should be small
        engine.step(1.0, 0.0, 0.0, &atmo, dt);
        assert!(
            engine.current_throttle < 0.1,
            "Jet engine should not respond instantly: {:.4}",
            engine.current_throttle
        );
    }

    #[test]
    fn thrust_on_sea_level_in_range() {
        let mut engine = JetEngine::f110_dry();
        let atmo = ConstantDensity::sea_level();
        let dt = TimeStep::constant(0.01);
        for _ in 0..500 {
            engine.step(1.0, 0.0, 0.0, &atmo, dt);
        }

        // F110 max thrust: ~76 kN
        assert!(
            engine.thrust() > 60_000.0 && engine.thrust() < 90_000.0,
            "Thrust max F110: {:.0} N (expected ~76 000 N)",
            engine.thrust()
        );
    }
}
