use crate::motor::{Motor, MotorArray};
use crate::time::TimeStep;
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct RotorParams {
    pub time_constant_s: f64, // [s]
    pub rotor_inertia: f64,   // [kg m^2]
    pub max_speed: f64,       // [rad/s]
    pub min_speed: f64,       // [rad/s]
}

impl RotorParams {
    pub fn mini3() -> Self {
        Self {
            time_constant_s: 0.04,
            rotor_inertia: 2.0e-5,
            max_speed: 1120.0,
            min_speed: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct QuadrotorRotors {
    pub params: RotorParams,
    current_speeds: MotorArray<f64>,
}

impl QuadrotorRotors {
    pub fn new(params: RotorParams) -> Self {
        Self {
            params,
            current_speeds: MotorArray::uniform(0.0),
        }
    }

    pub fn mini3() -> Self {
        Self::new(RotorParams::mini3())
    }

    pub fn at_hover(params: RotorParams, hover_speed: f64) -> Self {
        Self {
            params,
            current_speeds: MotorArray::uniform(hover_speed),
        }
    }

    pub fn speeds(&self) -> &MotorArray<f64> {
        &self.current_speeds
    }

    pub fn step(&mut self, commanded: &MotorArray<f64>, dt: TimeStep) {
        let alpha = (-dt.seconds() / self.params.time_constant_s).exp();
        let one_minus_alpha = 1.0 - alpha;

        for motor in Motor::ALL {
            let w_cmd = commanded[motor].clamp(self.params.min_speed, self.params.max_speed);
            let w_cur = self.current_speeds[motor];
            self.current_speeds[motor] = w_cur * alpha + w_cmd * one_minus_alpha;
        }
    }

    pub fn gyroscopic_torque(&self, aircraft_angular_velocity: &Vector3<f64>) -> Vector3<f64> {
        let jr = self.params.rotor_inertia;

        let omega_r: f64 = Motor::ALL
            .iter()
            .map(|&m| {
                let sigma = if m.is_clockwise() { 1.0 } else { -1.0 };
                sigma * self.current_speeds[m]
            })
            .sum();

        let h_rotors = Vector3::new(0.0, 0.0, jr * omega_r);

        aircraft_angular_velocity.cross(&h_rotors)
    }
}

pub fn body_drag(velocity_world: &Vector3<f64>, k_drag: f64) -> Vector3<f64> {
    let speed = velocity_world.norm();
    if speed < 1e-6 {
        return Vector3::zeros();
    }
    -velocity_world * (k_drag * speed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::TimeStep;

    #[test]
    fn engine_reaches_command() {
        let mut rotors = QuadrotorRotors::mini3();
        let cmd = MotorArray::uniform(500.0_f64);
        let dt = TimeStep::constant(0.005);

        // After time >> τ engine should reach command
        for _ in 0..1000 {
            rotors.step(&cmd, dt);
        }

        for m in Motor::ALL {
            assert!(
                (rotors.speeds()[m] - 500.0).abs() < 0.1,
                "Engine {:?} didn't reach command: {:.2}",
                m,
                rotors.speeds()[m]
            );
        }
    }

    #[test]
    fn engine_does_not_accelerate_instantly() {
        let mut rotors = QuadrotorRotors::mini3();
        let cmd = MotorArray::uniform(1000.0_f64);
        let dt = TimeStep::constant(0.005);

        // After one step (5ms << τ=40ms) engine is far from target
        rotors.step(&cmd, dt);
        for m in Motor::ALL {
            let w = rotors.speeds()[m];
            assert!(
                w < 500.0,
                "Engine {:?} accelerated too fast: {:.2} rad/s after 5ms",
                m,
                w
            );
            assert!(w > 0.0, "Engine {:?} should start accelerating", m);
        }
    }

    #[test]
    fn constant_time_correct() {
        // After time τ engine should reach 1-1/e ≈ 63.2% command
        let mut rotors = QuadrotorRotors::mini3();
        let tau = rotors.params.time_constant_s;
        let cmd = MotorArray::uniform(1000.0_f64);
        let dt = TimeStep::constant(0.001); // mały dt dla dokładności

        let steps = (tau / dt.seconds()).round() as usize;
        for _ in 0..steps {
            rotors.step(&cmd, dt);
        }

        let expected = 1000.0 * (1.0 - (-1.0_f64).exp()); // ≈ 632 rad/s
        for m in Motor::ALL {
            let w = rotors.speeds()[m];
            assert!(
                (w - expected).abs() < 5.0,
                "After time τ engine should reach {:.1}, got {:.1}",
                expected,
                w
            );
        }
    }

    #[test]
    fn constrained_to_max_speed() {
        let mut rotors = QuadrotorRotors::mini3();
        // Command above maximum
        let cmd = MotorArray::uniform(9999.0_f64);
        let dt = TimeStep::constant(0.005);

        for _ in 0..10000 {
            rotors.step(&cmd, dt);
        }

        for m in Motor::ALL {
            assert!(
                rotors.speeds()[m] <= rotors.params.max_speed + 1.0,
                "Engine {:?} exceeded max_speed: {:.2}",
                m,
                rotors.speeds()[m]
            );
        }
    }

    #[test]
    fn gyroscopic_torque_zero_on_hover() {
        // At symmetric hover (all engines equal) and no rotation
        // gyroscopic torque = 0
        let mut rotors = QuadrotorRotors::mini3();
        let hover_speed = 632.5;
        let cmd = MotorArray::uniform(hover_speed);
        let dt = TimeStep::constant(0.001);
        for _ in 0..10000 {
            rotors.step(&cmd, dt);
        }

        let omega_aircraft = nalgebra::Vector3::zeros();
        let tau_gyro = rotors.gyroscopic_torque(&omega_aircraft);

        assert!(
            tau_gyro.norm() < 1e-10,
            "At hover, gyroscopic torque should be zero"
        );
    }

    #[test]
    fn gyroscopic_torque_on_pitch_rotation() {
        // At asymmetric engines and pitch rotation
        // gyroscopic torque should appear
        let mut rotors = QuadrotorRotors::mini3();
        // Asymmetric speeds — like during yaw change
        let mut cmd = MotorArray::uniform(632.5_f64);
        use Motor::*;
        cmd[FrontRight] += 100.0;
        cmd[RearLeft] += 100.0;

        let dt = TimeStep::constant(0.001);
        for _ in 0..10000 {
            rotors.step(&cmd, dt);
        }

        // Rotation pitch (axis Y)
        let omega = nalgebra::Vector3::new(0.0, 1.0, 0.0);
        let tau_gyro = rotors.gyroscopic_torque(&omega);

        // Should be a nonzero gyroscopic torque
        assert!(
            tau_gyro.norm() > 1e-6,
            "At asymmetric engines and pitch rotation, \
             gyroscopic torque should be nonzero: {:?}",
            tau_gyro
        );
    }

    #[test]
    fn drag_opposes_velocity() {
        let v = nalgebra::Vector3::new(10.0, 0.0, 0.0);
        let drag = body_drag(&v, 0.1);
        // Drag should be in the opposite direction of velocity
        assert!(drag.x < 0.0, "Drag should oppose velocity");
        assert!(drag.y.abs() < 1e-10);
        assert!(drag.z.abs() < 1e-10);
    }

    #[test]
    fn drag_scales_with_v_squared() {
        let v1 = nalgebra::Vector3::new(10.0, 0.0, 0.0);
        let v2 = nalgebra::Vector3::new(20.0, 0.0, 0.0);
        let drag1 = body_drag(&v1, 0.1);
        let drag2 = body_drag(&v2, 0.1);
        // F ~ v² → double velocity = four times drag
        let ratio = drag2.x / drag1.x;
        assert!(
            (ratio - 4.0).abs() < 0.01,
            "Drag should scale with v², ratio={:.3}",
            ratio
        );
    }
}
