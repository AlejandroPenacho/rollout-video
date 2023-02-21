mod algorithm;
mod warehouse;

use warehouse::{Warehouse, WarehouseSim};

use nannou::prelude::*;

use std::collections::HashSet;

fn main() {
    nannou::app(initialize_model)
        .update(update)
        .simple_window(view)
        .run();
}

const DEFAULT_SPEED: f32 = 2.0;

const Y_POS: [f32; 5] = [-400.0, -200.0, 0.0, 200.0, 400.0];
const X_POS: [f32; 4] = [-200.0, 0.0, 200.0, 400.0];
const LOOKAHEAD_VISUAL_SIZE: f32 = 100.0;

struct Model {
    wait_time: Option<f32>,
    warehouse: Warehouse,
    base_waresim: WarehouseSim,
    base_cost: i32,
    alternatives: Vec<Option<WarehouseSim>>,
    alternative_paths: Vec<Option<WarehouseSim>>,
    robot_in_improvement: usize,
}

enum Stage {
    InitialWait,
    GenerateLookaheads,
    CreateWarehouses,
    AddRollout,
    Simulate,
    PickBest,
}

impl Model {
    fn generate_alternatives(&mut self) {
        let warehouse = &self.warehouse;
        let paths = self.base_waresim.get_paths();
        let alternatives: Vec<Option<WarehouseSim>> =
            algorithm::improve_policy(warehouse, paths, self.robot_in_improvement)
                .into_iter()
                .enumerate()
                .map(|(i, new_paths)| match new_paths {
                    Some(new_paths) => {
                        let mut drawhouse = WarehouseSim::new(
                            warehouse.clone(),
                            (X_POS[2], Y_POS[i]),
                            20.0,
                            new_paths,
                            DEFAULT_SPEED,
                        );
                        drawhouse.toggle_running(false);
                        Some(drawhouse)
                    }
                    None => None,
                })
                .collect();
        self.alternatives = alternatives;
        self.wait_time = Some(1.0);

        self.alternative_paths = self
            .alternatives
            .iter()
            .enumerate()
            .map(|(i, w)| {
                let Some(w) = w else { return None };
                let mut w: WarehouseSim = w.clone();
                w.adjust_for_path((X_POS[1], Y_POS[i]), LOOKAHEAD_VISUAL_SIZE);
                Some(w)
            })
            .collect();
    }

    fn check_completion(&self) -> bool {
        self.alternatives
            .iter()
            .all(|w| w.as_ref().map_or(true, |w| w.is_finished()))
    }

    fn save_best_policy(&mut self) {
        let mut best_index = self
            .alternatives
            .iter()
            .enumerate()
            .filter_map(|(i, x)| x.as_ref().map(|w| (i, w.get_current_cost())))
            .min_by_key(|(_, x)| *x)
            .unwrap()
            .0;

        let no_change_index = self
            .alternatives
            .iter()
            .position(|p| {
                let Some(p) = p.as_ref() else { return false };
                let original_continuation =
                    self.base_waresim.get_paths()[self.robot_in_improvement].a_star[0];
                let alternative_continuation =
                    p.get_paths()[self.robot_in_improvement].lookahead[0];
                original_continuation == alternative_continuation
            })
            .unwrap();

        let improved_cost = self.alternatives[best_index]
            .as_ref()
            .unwrap()
            .get_current_cost();

        if improved_cost == self.base_cost {
            println!("Prev index: {}", best_index);
            best_index = no_change_index;
            println!("New index: {}", best_index);
        }

        println!("The Best Cost: {}", improved_cost);

        let mut best_policy = self.alternatives[best_index]
            .as_ref()
            .unwrap()
            .get_paths()
            .to_owned();
        best_policy.iter_mut().for_each(|p| p.integrate_lookahead());

        self.base_cost = improved_cost;

        self.base_waresim = WarehouseSim::new(
            self.warehouse.clone(),
            (X_POS[0], Y_POS[2]),
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
        (X_POS[0], Y_POS[2]),
        20.0,
        paths,
        DEFAULT_SPEED,
    );

    let mut model = Model {
        warehouse,
        base_cost: 0,
        base_waresim: drawhouse,
        alternatives: vec![],
        alternative_paths: vec![],
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
        .filter_map(|x| x.as_mut())
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
            .filter_map(|x| x.as_mut())
            .for_each(|w| w.toggle_running(true));
        model.wait_time = None;
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    frame.clear(GRAY);
    let draw = app.draw();
    draw.background().color(GRAY);

    model.base_waresim.draw(&draw);
    model
        .alternatives
        .iter()
        .enumerate()
        .filter_map(|(i, x)| x.as_ref().map(|x| (i, x)))
        .for_each(|(i, w)| {
            w.draw(&draw);
            w.draw_cost((X_POS[3], Y_POS[i]), &draw);
        });

    let lookahead_turn = model.base_waresim.get_paths()[model.robot_in_improvement]
        .improved_path
        .len();

    const LOOKAHEADS: [&str; 5] = ["Wait", "Go up", "Go right", "Go down", "Go left"];
    model
        .alternative_paths
        .iter()
        .enumerate()
        .for_each(|(i, w)| {
            draw.text(LOOKAHEADS[i])
                .x_y(X_POS[1], Y_POS[i] + LOOKAHEAD_VISUAL_SIZE / 2.0);

            if let Some(w) = w {
                w.draw_robot(model.robot_in_improvement, 0.0, &draw);
                w.draw_robot_path(model.robot_in_improvement, 0.0, Some(lookahead_turn), &draw);
            } else {
                draw.text("Unfeasible").x_y(X_POS[1], Y_POS[i]);
            }
        });

    draw.to_frame(app, &frame).unwrap();
}
