mod algorithm;
mod warehouse;

use warehouse::{AgentPath, Warehouse, WarehouseSim};

use nannou::prelude::*;

use core::panic;
use std::collections::HashSet;

fn main() {
    nannou::app(initialize_model)
        .update(update)
        .simple_window(view)
        .run();
}

const DEFAULT_SPEED: f32 = 2.0;
const Y_POS: [f32; 5] = [-200.0, 0.0, 200.0, 400.0, 600.0];

struct Model {
    wait_time: Option<f32>,
    warehouse: Warehouse,
    base_waresim: WarehouseSim,
    alternatives: Vec<WarehouseSim>,
    robot_in_improvement: usize,
}

impl Model {
    fn generate_alternatives(&mut self) {
        let warehouse = &self.warehouse;
        let paths = self.base_waresim.get_paths();
        let alternatives: Vec<WarehouseSim> =
            algorithm::improve_policy(warehouse, paths, self.robot_in_improvement)
                .into_iter()
                .enumerate()
                .map(|(i, new_paths)| {
                    let mut drawhouse = WarehouseSim::new(
                        warehouse.clone(),
                        (200.0, Y_POS[i]),
                        20.0,
                        new_paths,
                        DEFAULT_SPEED,
                    );
                    drawhouse.toggle_running(false);
                    drawhouse
                })
                .collect();
        self.alternatives = alternatives;
        self.wait_time = Some(1.0);
    }

    fn check_completion(&self) -> bool {
        self.alternatives.iter().all(|w| w.is_finished())
    }

    fn save_best_policy(&mut self) {
        let mut best_policy = self
            .alternatives
            .iter()
            .min_by_key(|w| w.get_current_cost())
            .unwrap()
            .get_paths()
            .to_owned();

        best_policy.iter_mut().for_each(|p| p.integrate_lookahead());

        for path in best_policy.iter() {
            println!("{:?}", path)
        }

        self.base_waresim = WarehouseSim::new(
            self.warehouse.clone(),
            (-200.0, 50.0),
            20.0,
            best_policy,
            DEFAULT_SPEED,
        );

        self.robot_in_improvement =
            (self.robot_in_improvement + 1) % self.base_waresim.get_paths().len();
    }
}

fn initialize_model(_app: &App) -> Model {
    let mut walls = HashSet::new();
    walls.insert((0, 3));
    walls.insert((1, 3));
    walls.insert((3, 3));

    let endpoints = [((0, 0), (4, 5)), ((2, 7), (7, 3)), ((9, 7), (2, 3))];

    let warehouse = Warehouse::new((12, 8), walls);
    let paths = algorithm::initialize_paths(&warehouse, &endpoints);

    let drawhouse = WarehouseSim::new(
        warehouse.clone(),
        (-200.0, 50.0),
        20.0,
        paths,
        DEFAULT_SPEED,
    );

    let mut model = Model {
        warehouse,
        base_waresim: drawhouse,
        alternatives: vec![],
        robot_in_improvement: 0,
        wait_time: None,
    };
    model.generate_alternatives();

    model
}

fn update(_app: &App, model: &mut Model, update: Update) {
    model
        .alternatives
        .iter_mut()
        .for_each(|w| w.update_time(update.since_last.as_secs_f32()));

    if let Some(t) = model.wait_time.as_mut() {
        *t -= update.since_last.as_secs_f32();
    };

    if model.check_completion() {
        model.save_best_policy();
        model.generate_alternatives();
    }

    if model.wait_time.map_or(false, |t| t < 0.0) {
        model
            .alternatives
            .iter_mut()
            .for_each(|w| w.toggle_running(true));
        model.wait_time = None;
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(GRAY);

    model.base_waresim.draw(&draw);
    model.alternatives.iter().for_each(|w| w.draw(&draw));

    draw.to_frame(app, &frame).unwrap();
}
