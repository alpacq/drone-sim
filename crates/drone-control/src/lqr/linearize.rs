use drone_model::{
    state::DroneState,
    vehicle::{KnownActuatorInput, StateDot, VehicleModel},
};
use nalgebra::{DMatrix, DVector, Quaternion, UnitQuaternion, Vector3};

const EPSILON: f64 = 1.49e-8_f64;

pub fn state_to_vec(state: &DroneState) -> DVector<f64> {
    let q = state.orientation.quaternion();
    DVector::from_vec(vec![
        state.position.x,
        state.position.y,
        state.position.z,
        state.velocity.x,
        state.velocity.y,
        state.velocity.z,
        state.angular_velocity.x,
        state.angular_velocity.y,
        state.angular_velocity.z,
        q.i,
        q.j,
        q.k,
        q.w,
    ])
}

pub fn vec_to_state(vec: &DVector<f64>, template: &DroneState) -> DroneState {
    DroneState {
        position: Vector3::new(vec[0], vec[1], vec[2]),
        velocity: Vector3::new(vec[3], vec[4], vec[5]),
        angular_velocity: Vector3::new(vec[6], vec[7], vec[8]),
        // state_to_vec stores [q.i, q.j, q.k, q.w] at indices 9-12.
        // Quaternion::new takes (w, i, j, k), so w is at index 12.
        orientation: UnitQuaternion::from_quaternion(Quaternion::new(
            vec[12], vec[9], vec[10], vec[11],
        )),
        actuator_state: template.actuator_state.clone(),
    }
}

pub fn input_to_vec(input: &KnownActuatorInput) -> DVector<f64> {
    match input {
        KnownActuatorInput::Quadrotor(speeds) => {
            use drone_model::motor::Motor;
            DVector::from_vec(vec![
                speeds[Motor::FrontRight],
                speeds[Motor::FrontLeft],
                speeds[Motor::RearLeft],
                speeds[Motor::RearRight],
            ])
        }
        KnownActuatorInput::FixedWing {
            throttle,
            aileron,
            elevator,
            rudder,
        } => DVector::from_vec(vec![*throttle, *aileron, *elevator, *rudder]),
    }
}

pub fn vec_to_input(v: &DVector<f64>, template: &KnownActuatorInput) -> KnownActuatorInput {
    match template {
        KnownActuatorInput::Quadrotor(_) => {
            use drone_model::motor::{Motor, MotorArray};
            let mut speeds = MotorArray::uniform(0.0);
            speeds[Motor::FrontRight] = v[0];
            speeds[Motor::FrontLeft] = v[1];
            speeds[Motor::RearLeft] = v[2];
            speeds[Motor::RearRight] = v[3];
            KnownActuatorInput::Quadrotor(speeds)
        }
        KnownActuatorInput::FixedWing { .. } => KnownActuatorInput::FixedWing {
            throttle: v[0],
            aileron: v[1],
            elevator: v[2],
            rudder: v[3],
        },
    }
}

fn state_dot_to_vec(dot: &StateDot) -> DVector<f64> {
    DVector::from_vec(vec![
        dot.velocity.x,
        dot.velocity.y,
        dot.velocity.z,
        dot.acceleration.x,
        dot.acceleration.y,
        dot.acceleration.z,
        dot.angular_acceleration.x,
        dot.angular_acceleration.y,
        dot.angular_acceleration.z,
        dot.orientation_dot.i,
        dot.orientation_dot.j,
        dot.orientation_dot.k,
        dot.orientation_dot.w,
    ])
}

#[derive(Debug, Clone)]
pub struct LinearizedModel {
    pub a: DMatrix<f64>, // state dynamics matrix [13x13] 13 instead of 12 because quaternion has 4 components but only 3 degrees of freedom
    pub b: DMatrix<f64>, // input matrix [13×m], m - number of inputs (4)
    pub x0: DVector<f64>, // working point - state to be linearized around
    pub u0: DVector<f64>, // working point - input to be linearized around
}

pub fn linearize(
    model: &dyn VehicleModel,
    state0: &DroneState,
    input0: &KnownActuatorInput,
) -> LinearizedModel {
    let n = 13_usize;
    let m = input_to_vec(input0).len();

    let x0 = state_to_vec(state0);
    let u0 = input_to_vec(input0);

    let mut a = DMatrix::zeros(n, n);

    for j in 0..n {
        let mut x_plus = x0.clone();
        let mut x_minus = x0.clone();
        x_plus[j] += EPSILON;
        x_minus[j] -= EPSILON;

        let state_plus = vec_to_state(&x_plus, state0);
        let state_minus = vec_to_state(&x_minus, state0);

        let f_plus = state_dot_to_vec(&model.derivatives(&state_plus, input0));
        let f_minus = state_dot_to_vec(&model.derivatives(&state_minus, input0));

        let col = (f_plus - f_minus) / (2.0 * EPSILON);
        a.set_column(j, &col);
    }

    let mut b = DMatrix::zeros(n, m);

    for j in 0..m {
        let mut u_plus = u0.clone();
        let mut u_minus = u0.clone();
        u_plus[j] += EPSILON;
        u_minus[j] -= EPSILON;

        let input_plus = vec_to_input(&u_plus, input0);
        let input_minus = vec_to_input(&u_minus, input0);

        let f_plus = state_dot_to_vec(&model.derivatives(state0, &input_plus));
        let f_minus = state_dot_to_vec(&model.derivatives(state0, &input_minus));

        let col = (f_plus - f_minus) / (2.0 * EPSILON);
        b.set_column(j, &col);
    }

    LinearizedModel { a, b, x0, u0 }
}

pub fn discretize_euler(
    a: &DMatrix<f64>,
    b: &DMatrix<f64>,
    dt: f64,
) -> (DMatrix<f64>, DMatrix<f64>) {
    let n = a.nrows();
    let ad = DMatrix::identity(n, n) + a * dt;
    let bd = b * dt;
    (ad, bd)
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_model::{state::DroneState, vehicle::quadrotor::QuadrotorModel};
    use nalgebra::{UnitQuaternion, Vector3};

    fn hover_state() -> DroneState {
        DroneState {
            position: Vector3::new(0.0, 0.0, 10.0),
            velocity: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            actuator_state: None,
        }
    }

    #[test]
    fn round_trip_state_vec() {
        let state = hover_state();
        let vec = state_to_vec(&state);
        let state2 = vec_to_state(&vec, &state);

        assert!((state.position - state2.position).norm() < 1e-10);
        assert!((state.velocity - state2.velocity).norm() < 1e-10);
        assert!((state.angular_velocity - state2.angular_velocity).norm() < 1e-10);
    }

    #[test]
    fn a_matrix_has_correct_dimensions() {
        let model = QuadrotorModel::mini3_simple();
        let state = hover_state();
        let input = model.equilibrium_input();
        let lm = linearize(&model, &state, &input);

        assert_eq!(lm.a.nrows(), 13);
        assert_eq!(lm.a.ncols(), 13);
        assert_eq!(lm.b.nrows(), 13);
        assert_eq!(lm.b.ncols(), 4); // 4 silniki
    }

    #[test]
    fn b_matrix_nonzero() {
        // B should be nonzero — control input affects derivatives
        let model = QuadrotorModel::mini3_simple();
        let state = hover_state();
        let input = model.equilibrium_input();
        let lm = linearize(&model, &state, &input);

        assert!(
            lm.b.norm() > 0.1,
            "B matrix should be nonzero: norm = {:.4}",
            lm.b.norm()
        );
    }

    #[test]
    fn euler_discretization_correct() {
        // Simple scalar: ẋ = -x → Ad = 1 - dt
        let a = DMatrix::from_element(1, 1, -1.0);
        let b = DMatrix::from_element(1, 1, 0.0);
        let dt = 0.01;
        let (ad, _) = discretize_euler(&a, &b, dt);
        assert!((ad[(0, 0)] - 0.99).abs() < 1e-10);
    }
}
