//! Przechwytywanie planów predykcyjnych MPC podczas symulacji SITL.
//!
//! W każdym kroku MPC oblicza sekwencję sterowań optymalnych na horyzoncie N.
//! Ta sekwencja wyznacza **planowaną trajektorię** – gdzie kontroler "spodziewa
//! się" znaleźć drona w ciągu najbliższych N·dt_pred sekund.
//!
//! [`run_capturing_horizons`] uruchamia symulację scenariusza i co
//! `capture_interval_s` sekund zapisuje aktualny plan wysokości z MPC do
//! wektora [`HorizonSnapshot`].  Snapshoty mogą następnie zostać
//! zwizualizowane funkcją [`crate::drone_plot::plot_mpc_horizon`].

use anyhow::Result;
use drone_control::controller::Controller;
use drone_control::target::FlightTarget;
use drone_model::{state::DroneState, time::TimeStep, vehicle::VehicleModel};
use drone_sim::runner::{SimConfig, SimFrame};
use drone_sim::integrator::{Integrator as _, RK4};
use nalgebra::{UnitQuaternion, Vector3};

use crate::disturbance::Disturbance;
use crate::runner::{ControllerFactory, scenario_to_flight_target};
use crate::scenario::Scenario;

/// Jeden "snapshot" planu MPC w chwili `sim_time`.
///
/// `pred_z[0]` to aktualna wysokość drona (punkt startowy planu).
/// `pred_z[k]` to przewidywana wysokość po `k` krokach predykcji
/// (każdy krok odpowiada `pred_dt_s` sekundom).
#[derive(Debug, Clone)]
pub struct HorizonSnapshot {
    /// Czas symulacji, w którym wykonano snapshot [s].
    pub sim_time: f64,
    /// Krok predykcji MPC [s] — odległość czasowa między sąsiednimi pred_z.
    pub pred_dt_s: f64,
    /// Predykcja wysokości: `pred_z[k]` = planowana wysokość po `k` krokach.
    pub pred_z: Vec<f64>,
}

impl HorizonSnapshot {
    /// Zwraca czas (w sekundach od `sim_time`) odpowiadający pred_z[k].
    pub fn pred_time(&self, k: usize) -> f64 {
        self.sim_time + k as f64 * self.pred_dt_s
    }
}

/// Uruchamia scenariusz i przechwytuje snapshoty planów MPC.
///
/// Funkcja działa jak [`crate::runner::run_scenario`], ale co
/// `capture_interval_s` sekund wywołuje [`Controller::planned_z_horizon`]
/// i zapisuje wynik do listy [`HorizonSnapshot`].
///
/// Jeśli kontroler nie obsługuje `planned_z_horizon` (np. PID, LQR),
/// snapshoty są puste – funkcja działa poprawnie dla każdego kontrolera.
///
/// # Argumenty
/// * `pred_dt_s` – długość jednego kroku predykcji MPC [s]; potrzebna
///   do przeliczenia indeksu horyzontu na czas.
pub fn run_capturing_horizons(
    scenario: &Scenario,
    model: &dyn VehicleModel,
    factory: &ControllerFactory,
    capture_interval_s: f64,
    pred_dt_s: f64,
) -> Result<(Vec<SimFrame>, Vec<HorizonSnapshot>)> {
    let dt = TimeStep::new(scenario.dt_s)
        .map_err(|e| anyhow::anyhow!("Invalid dt: {}", e))?;

    let [roll, pitch, yaw] = scenario.initial.attitude_deg;
    let orientation = UnitQuaternion::from_euler_angles(
        roll.to_radians(),
        pitch.to_radians(),
        yaw.to_radians(),
    );
    let mut state = DroneState {
        position: Vector3::from(scenario.initial.position),
        velocity: Vector3::from(scenario.initial.velocity),
        angular_velocity: Vector3::zeros(),
        orientation,
        actuator_state: None,
    };

    let disturbances: Vec<Box<dyn Disturbance>> = scenario
        .disturbances
        .iter()
        .cloned()
        .map(|d| d.into_disturbance())
        .collect();

    let config = SimConfig { dt, duration: scenario.duration_s };
    let target = scenario_to_flight_target(&scenario.target);
    let mut controller = factory(model)?;

    let steps = (config.duration / config.dt.seconds()).ceil() as usize;
    let mut frames: Vec<SimFrame> = Vec::with_capacity(steps + 1);
    let mut snapshots: Vec<HorizonSnapshot> = Vec::new();

    frames.push(SimFrame { time: 0.0, state: state.clone() });
    let mut time = 0.0_f64;
    let mut next_capture = 0.0_f64;

    for _ in 0..steps {
        for d in &disturbances {
            if d.is_active(time) {
                d.apply(&mut state, model, config.dt);
            }
        }

        let input = controller.update(&state, &target, config.dt);

        // Przechwytuj plan MPC (jeśli dostępny)
        if time >= next_capture - 1e-9 {
            if let Some(plan) = controller.planned_z_horizon() {
                snapshots.push(HorizonSnapshot {
                    sim_time: time,
                    pred_dt_s,
                    pred_z: plan.to_vec(),
                });
            }
            next_capture += capture_interval_s;
        }

        model.step_actuators(&mut state, &input, config.dt);
        state = RK4.step(model, &state, &input, config.dt);
        time += config.dt.seconds();

        frames.push(SimFrame { time, state: state.clone() });
    }

    Ok((frames, snapshots))
}
