use crate::{
    controller::Controller,
    inner_loop::InnerLoop,
    mixer::{AttitudeCommand, Mixer},
    profiler::VelocityProfiler,
    target::FlightTarget,
};
use drone_model::{
    math::euler::quat_to_euler,
    state::DroneState,
    time::TimeStep,
    vehicle::{KnownActuatorInput, VehicleModel},
};
/// Cascade flight controller - full XYZ + yaw
///
/// Three cascade levels:
/// - 1. Position -> Profiler -> target velocity
/// - 2. Velocity -> InnerLoop -> target angle / throttle
/// - 3. Angle -> InnerLoop -> motor speeds through mixer
/// Generic with profiler (P) and inner loop (I)
pub struct CascadeController<P, I>
where
    P: VelocityProfiler,
    I: InnerLoop,
{
    /// outer loop: position -> velocity
    profiler_z: P,
    profiler_xy: P,

    /// middle loop: velocity -> angle / throttle
    vel_loop_z: I, // vZ -> throttle_delta
    vel_loop_x: I, // vX -> target pitch,
    vel_loop_y: I, // vY -> target roll,

    /// inner loop: angle -> motor speeds
    att_loop_roll: I,
    att_loop_pitch: I,
    att_loop_yaw: I,

    mixer: Box<dyn Mixer>,

    /// max tilt angle for XY control [rad]
    /// ~20° = 0.35rad — safe limit for Mini 3
    pub max_tilt_rad: f64,
}

impl<P, I> CascadeController<P, I>
where
    P: VelocityProfiler,
    I: InnerLoop,
{
    pub fn new(
        mixer: Box<dyn Mixer>,
        profiler_z: P,
        profiler_xy: P,
        vel_loop_z: I,
        vel_loop_x: I,
        vel_loop_y: I,
        att_loop_roll: I,
        att_loop_pitch: I,
        att_loop_yaw: I,
    ) -> Self {
        Self {
            profiler_z,
            profiler_xy,
            vel_loop_z,
            vel_loop_x,
            vel_loop_y,
            att_loop_roll,
            att_loop_pitch,
            att_loop_yaw,
            mixer,
            max_tilt_rad: 0.35,
        }
    }
}

impl<P, I> Controller for CascadeController<P, I>
where
    P: VelocityProfiler,
    I: InnerLoop,
{
    fn update(
        &mut self,
        state: &DroneState,
        target: &FlightTarget,
        dt: TimeStep,
    ) -> KnownActuatorInput {
        let euler = quat_to_euler(&state.orientation);

        // outer loop
        let (target_vx, target_vy, target_vz) = match &target.position {
            Some(pos) => {
                let err = pos - state.position;
                (
                    self.profiler_xy.compute(err.x),
                    self.profiler_xy.compute(err.y),
                    self.profiler_z.compute(err.z),
                )
            }
            None => (0.0, 0.0, 0.0),
        };

        // middle loop
        let err_vz = target_vz - state.velocity.z;
        let throttle_delta = self.vel_loop_z.compute(err_vz, dt);

        let err_vx = target_vx - state.velocity.x;
        let target_pitch = -self
            .vel_loop_x
            .compute(err_vx, dt)
            .clamp(-self.max_tilt_rad, self.max_tilt_rad);

        let err_vy = target_vy - state.velocity.y;
        let target_roll = self
            .vel_loop_y
            .compute(err_vy, dt)
            .clamp(-self.max_tilt_rad, self.max_tilt_rad);

        // inner loop
        let cmd_roll = self.att_loop_roll.compute(target_roll - euler.roll, dt);
        let cmd_pitch = self.att_loop_pitch.compute(target_pitch - euler.pitch, dt);
        let cmd_yaw = match target.yaw {
            Some(yaw) => self
                .att_loop_yaw
                .compute(normalize_angle(yaw - euler.yaw), dt),
            None => 0.0,
        };

        let eq = self.mixer.equilibrium_command();
        let throttle = (eq.throttle + throttle_delta).clamp(0.0, 1.0);

        let cmd = AttitudeCommand {
            throttle,
            roll: cmd_roll,
            pitch: cmd_pitch,
            yaw: cmd_yaw,
        };

        self.mixer.mix(&cmd)
    }

    fn reset(&mut self) {
        self.vel_loop_z.reset();
        self.vel_loop_x.reset();
        self.vel_loop_y.reset();
        self.att_loop_roll.reset();
        self.att_loop_pitch.reset();
        self.att_loop_yaw.reset();
    }

    fn name(&self) -> &str {
        "CascadeController"
    }
}

fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

pub fn make_cascade(
    model: &dyn VehicleModel,
) -> CascadeController<crate::profiler::sqrt::SqrtProfiler, crate::inner_loop::pid_loop::PidLoop> {
    use crate::inner_loop::pid_loop::PidLoop;
    use crate::mixer::fixed_wing::FixedWingMixer;
    use crate::mixer::quadrotor::QuadrotorMixer;
    use crate::profiler::sqrt::SqrtProfiler;

    let mixer: Box<dyn crate::mixer::Mixer> = match model.equilibrium_input() {
        drone_model::vehicle::KnownActuatorInput::Quadrotor(_) => {
            Box::new(QuadrotorMixer::from_equilibrium(model.equilibrium_input()))
        }
        drone_model::vehicle::KnownActuatorInput::FixedWing { .. } => {
            Box::new(FixedWingMixer::from_equilibrium(model.equilibrium_input()))
        }
    };

    CascadeController::new(
        mixer,
        SqrtProfiler::for_altitude(),
        SqrtProfiler::for_horizontal(),
        PidLoop::new(0.3, 0.1, 0.0, 0.45, 0.45),
        PidLoop::new(0.4, 0.05, 0.0, 0.5, 0.35),
        PidLoop::new(0.4, 0.05, 0.0, 0.5, 0.35),
        PidLoop::new(4.0, 0.0, 0.2, 1.0, 1.0),
        PidLoop::new(4.0, 0.0, 0.2, 1.0, 1.0),
        PidLoop::new(2.0, 0.1, 0.0, 0.5, 0.5),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{state::DroneState, time::TimeStep, vehicle::quadrotor::QuadrotorModel};
    use nalgebra::{UnitQuaternion, Vector3};

    fn dt() -> TimeStep {
        TimeStep::constant(0.005)
    }

    fn ground_state() -> DroneState {
        DroneState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
        }
    }

    #[test]
    fn hover_in_place_does_hover() {
        let model = QuadrotorModel::mini3();
        let mut ctrl = make_cascade(&model);
        let hover_speed = match model.equilibrium_input() {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };

        let target = FlightTarget::altitude(0.0);
        let input = ctrl.update(&ground_state(), &target, dt());

        let avg = match input {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };

        assert!(
            (avg - hover_speed).abs() < 1.0,
            "avg={:.1} hover={:.1}",
            avg,
            hover_speed
        );
    }

    #[test]
    fn target_above_increases_throttle() {
        let model = QuadrotorModel::mini3();
        let mut ctrl = make_cascade(&model);
        let hover_speed = match model.equilibrium_input() {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };

        let target = FlightTarget::altitude(5.0);
        let input = ctrl.update(&ground_state(), &target, dt());

        let avg = match input {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };

        assert!(avg > hover_speed, "avg={:.1} hover={:.1}", avg, hover_speed);
    }

    #[test]
    fn normalize_angle_works() {
        use std::f64::consts::PI;
        assert!((normalize_angle(0.0)).abs() < 1e-10);
        assert!((normalize_angle(PI + 0.1) - (-PI + 0.1)).abs() < 1e-10);
        assert!((normalize_angle(-PI - 0.1) - (PI - 0.1)).abs() < 1e-10);
    }

    #[test]
    fn reset_zeroes_controllers() {
        let model = QuadrotorModel::mini3();
        let mut ctrl = make_cascade(&model);

        let target = FlightTarget::full(10.0, 10.0, 10.0, 1.0);
        for _ in 0..100 {
            ctrl.update(&ground_state(), &target, dt());
        }

        ctrl.reset();

        let hover_speed = match model.equilibrium_input() {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };
        let target_zero = FlightTarget::altitude(0.0);
        let input = ctrl.update(&ground_state(), &target_zero, dt());
        let avg = match input {
            KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
            _ => panic!(),
        };

        assert!(
            (avg - hover_speed).abs() < 5.0,
            "After reset, avg={:.1} should be close to hover={:.1}",
            avg,
            hover_speed
        );
    }
}
