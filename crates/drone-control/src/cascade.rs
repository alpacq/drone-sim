// use crate::mixer::{AttitudeCommand, mix};
// use crate::pid::Pid;
// use drone_model::{
//     state::DroneState,
//     time::TimeStep,
//     vehicle::{KnownActuatorInput, VehicleModel},
// };

// // Max approach speed [m/s]
// const V_MAX: f64 = 1.0;
// // Deceleration used to shape the velocity profile [m/s²]
// // target_vz = sign(err) * min(sqrt(2 * BRAKE_ACCEL * |err|), V_MAX)
// // This guarantees the drone slows down naturally before the setpoint.
// const BRAKE_ACCEL: f64 = 1.5;

// // Z axis (altitude) cascade controller
// // Outer loop: sqrt velocity profiler (replaces linear position PID)
// // Inner loop: velocity PI — NO D term (D on velocity causes limit-cycling
// //   because dω/dt from thrust changes is ~20 m/s², which with any kd > 0.02
// //   immediately saturates the output every step)
// pub struct AltitudeController {
//     pid_velocity: Pid,
//     hover_motor_speed: f64,
//     max_motor_speed: f64,
// }

// impl AltitudeController {
//     pub fn new(model: &dyn VehicleModel) -> Self {
//         // ω = √(m*g / (4*k_thrust))
//         let hover_motor_speed = match model.equilibrium_input() {
//             KnownActuatorInput::Quadrotor(speeds) => speeds.sum() / 4.0,
//             other => panic!(
//                 "AltitudeController only supports quadrotors, given: {:?}",
//                 other
//             ),
//         };
//         let max_motor_speed = hover_motor_speed * 1.77152318;

//         Self {
//             // kd=0: derivative on velocity causes limit-cycling (see module comment)
//             pid_velocity: Pid::new(0.3, 0.1, 0.0, 0.45, 0.45),
//             hover_motor_speed,
//             max_motor_speed,
//         }
//     }

//     pub fn update(
//         &mut self,
//         state: &DroneState,
//         target_z: f64,
//         dt: TimeStep,
//     ) -> KnownActuatorInput {
//         // Outer loop: sqrt velocity profile
//         // Velocity setpoint scales with sqrt(|error|) so the drone decelerates
//         // smoothly rather than arriving at full speed and overshooting.
//         let error_z = target_z - state.position.z;
//         let target_vz = error_z.signum() * (2.0 * BRAKE_ACCEL * error_z.abs()).sqrt().min(V_MAX);

//         // Inner loop: velocity PI
//         let error_vz = target_vz - state.velocity.z;
//         let throttle_delta = self.pid_velocity.update(error_vz, dt);

//         let hover_throttle = self.hover_motor_speed / self.max_motor_speed;
//         let throttle = (hover_throttle + throttle_delta).clamp(0.0, 1.0);

//         let cmd = AttitudeCommand {
//             throttle,
//             roll: 0.0,
//             pitch: 0.0,
//             yaw: 0.0,
//         };

//         KnownActuatorInput::Quadrotor(mix(&cmd, self.max_motor_speed))
//     }

//     pub fn reset(&mut self) {
//         self.pid_velocity.reset();
//     }
// }

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use drone_model::{state::DroneState, time::TimeStep, vehicle::quadrotor::QuadrotorModel};
//     use nalgebra::{UnitQuaternion, Vector3};

//     fn dt() -> TimeStep {
//         TimeStep::constant(0.01)
//     }

//     fn state_on_ground() -> DroneState {
//         DroneState {
//             position: Vector3::zeros(),
//             velocity: Vector3::zeros(),
//             orientation: UnitQuaternion::identity(),
//             angular_velocity: Vector3::zeros(),
//         }
//     }

//     fn hover_speed(model: &QuadrotorModel) -> f64 {
//         match model.equilibrium_input() {
//             KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
//             _ => panic!(),
//         }
//     }

//     #[test]
//     fn controller_gives_more_throttle_if_too_low() {
//         let model = QuadrotorModel::mini3();
//         let mut ctrl = AltitudeController::new(&model);
//         let input = ctrl.update(&state_on_ground(), 5.0, dt());

//         let avg_speed = match input {
//             KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
//             _ => panic!(),
//         };

//         assert!(
//             avg_speed > hover_speed(&model),
//             "avg={:.1} hover={:.1}",
//             avg_speed,
//             hover_speed(&model)
//         );
//     }

//     #[test]
//     fn controler_gives_less_throttle_if_too_high() {
//         let model = QuadrotorModel::mini3();
//         let mut ctrl = AltitudeController::new(&model);
//         let state = DroneState {
//             position: Vector3::new(0.0, 0.0, 10.0),
//             velocity: Vector3::zeros(),
//             orientation: UnitQuaternion::identity(),
//             angular_velocity: Vector3::zeros(),
//         };

//         let input = ctrl.update(&state, 5.0, dt());
//         let avg_speed = match input {
//             KnownActuatorInput::Quadrotor(s) => s.sum() / 4.0,
//             _ => panic!(),
//         };

//         assert!(
//             avg_speed < hover_speed(&model),
//             "avg={:.1} hover={:.1}",
//             avg_speed,
//             hover_speed(&model)
//         );
//     }

//     #[test]
//     fn diagnostics_hover_speed() {
//         let model = QuadrotorModel::mini3();
//         let ctrl = AltitudeController::new(&model);
//         let expected = hover_speed(&model);

//         println!(
//             "hover_motor_speed (kontroler): {:.2}",
//             ctrl.hover_motor_speed
//         );
//         println!("hover_motor_speed (model):     {:.2}", expected);

//         assert!(
//             (ctrl.hover_motor_speed - expected).abs() < 0.01,
//             "Discrepancy: {:.2} vs {:.2}",
//             ctrl.hover_motor_speed,
//             expected
//         );
//     }
// }
