// PID controller with anti-windup
#[derive(Debug, Clone)]
pub struct Pid {
    // controller parameters
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,

    // max absolute value of integral part (protects against integral windup)
    pub integral_limit: f64,

    // max absolute value of output
    pub output_limit: f64,

    // internal state
    integral: f64,
    prev_error: f64,
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    // computes controller's output for given error and timestep
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        // P part
        let p = self.kp * error;

        // I part
        self.integral += error * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
        let i = self.ki * self.integral;

        // D part
        let d = if dt > 1e-10 {
            self.kd * (error - self.prev_error) / dt
        } else {
            0.0
        };
        self.prev_error = error;

        (p + i + d).clamp(-self.output_limit, self.output_limit)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn p_only_reduces_error() {
        let mut pid = Pid::new(2.0, 0.0, 0.0, 100.0, 100.0);
        let output = pid.update(1.0, 0.01);
        // Kp=2, błąd=1 → wyjście=2
        assert!((output - 2.0).abs() < 1e-10);
    }

    #[test]
    fn integral_windup_is_limited() {
        let mut pid = Pid::new(0.0, 1.0, 0.0, 1.0, 100.0);
        // Wiele kroków z dużym błędem
        for _ in 0..10000 {
            pid.update(100.0, 0.01);
        }
        // Integral nie może przekroczyć integral_limit
        // Ki=1, integral_limit=1 → max wyjście z I = 1.0
        let output = pid.update(0.0, 0.01);
        assert!(output.abs() <= 1.0 + 1e-10, "Windup! output={}", output);
    }

    #[test]
    fn d_slows_down() {
        let mut pid = Pid::new(0.0, 0.0, 1.0, 100.0, 100.0);
        let dt = 0.01;
        // Pierwszy krok: błąd=1, prev_error=0 → d = (1-0)/dt = 100
        let out1 = pid.update(1.0, dt);
        // Drugi krok: błąd=1, prev_error=1 → d = (1-1)/dt = 0
        let out2 = pid.update(1.0, dt);
        assert!(
            out1.abs() > out2.abs(),
            "D should be bigger with changing error"
        );
    }

    #[test]
    fn reset_zeroes_state() {
        let mut pid = Pid::new(0.0, 1.0, 0.0, 100.0, 100.0);
        pid.update(10.0, 0.1); // nabuduj integral
        pid.reset();
        let output = pid.update(0.0, 0.1); // zero błędu po resecie
        assert!(output.abs() < 1e-10, "After reset output should be 0");
    }
}
