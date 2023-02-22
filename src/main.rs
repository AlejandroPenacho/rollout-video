mod algorithm;
mod warehouse;

use warehouse::{WareSimLocation, Warehouse, WarehouseSim};

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
    base_cost: i32,
    base_waresim: WarehouseSim,
    alternatives: Vec<Option<WarehouseSim>>,
    alternative_paths: Vec<Option<WarehouseSim>>,
    floating_paths: Vec<WarehouseSim>,
    robot_in_improvement: usize,
    stage: Stage,
}

#[derive(Clone, Copy)]
struct Stage {
    id: StageId,
    time: f32,
}

#[derive(Clone, Copy)]
enum StageId {
    InitialWait,
    TranslateCurrentPath,
    GenerateLookaheads,
    CreateWarehouses,
    AddRollout,
    Simulate,
    PickBest,
}

impl StageId {
    fn get_duration(&self) -> Option<f32> {
        match self {
            _ => Some(2.0),
        }
    }
}

impl Stage {
    fn get_final(stage_id: StageId) -> Stage {
        Stage {
            id: stage_id,
            time: stage_id
                .get_duration()
                .expect("Attempted to do something crazy"),
        }
    }
    fn get_start(stage_id: StageId) -> Stage {
        Stage {
            id: stage_id,
            time: 0.0,
        }
    }
}

const LOOKAHEAD_NAMES: [&str; 5] = ["Wait", "Go up", "Go right", "Go down", "Go left"];

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
                            WareSimLocation::new((X_POS[2], Y_POS[i]), 20.0),
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
            best_index = no_change_index;
        }

        let mut best_policy = self.alternatives[best_index]
            .as_ref()
            .unwrap()
            .get_paths()
            .to_owned();
        best_policy.iter_mut().for_each(|p| p.integrate_lookahead());

        self.base_cost = improved_cost;

        self.base_waresim = WarehouseSim::new(
            self.warehouse.clone(),
            WareSimLocation::new((X_POS[0], Y_POS[2]), 20.0),
            best_policy,
            DEFAULT_SPEED,
        );

        self.robot_in_improvement =
            (self.robot_in_improvement + 1) % self.base_waresim.get_paths().len();
    }
}

impl Model {
    fn draw(&self, stage: &Stage, drawing: &Draw) {
        let lookahead_turn = self.base_waresim.get_paths()[self.robot_in_improvement]
            .improved_path
            .len();

        match &stage.id {
            StageId::InitialWait => {
                self.base_waresim.draw(drawing);
            }
            StageId::TranslateCurrentPath => {
                self.draw(&Stage::get_final(StageId::InitialWait), drawing);
                self.floating_paths.iter().for_each(|w| {
                    w.draw_robot_path(
                        self.robot_in_improvement,
                        0.0,
                        Some(w.get_paths()[self.robot_in_improvement].lookahead.len()),
                        drawing,
                    );
                    w.draw_robot(self.robot_in_improvement, 0.0, drawing);
                })
            }
            StageId::GenerateLookaheads => {
                self.draw(&Stage::get_final(StageId::InitialWait), drawing);

                self.alternative_paths
                    .iter()
                    .enumerate()
                    .for_each(|(i, w)| {
                        drawing
                            .text(LOOKAHEAD_NAMES[i])
                            .x_y(X_POS[1], Y_POS[i] + LOOKAHEAD_VISUAL_SIZE / 2.0);

                        if let Some(w) = w {
                            w.draw_robot(self.robot_in_improvement, 0.0, drawing);
                            w.draw_robot_path(
                                self.robot_in_improvement,
                                0.0,
                                Some(lookahead_turn + 1),
                                drawing,
                            );
                        } else {
                            drawing.text("Unfeasible").x_y(X_POS[1], Y_POS[i]);
                        }
                    });
            }
            StageId::CreateWarehouses => {
                self.draw(&Stage::get_final(StageId::GenerateLookaheads), drawing);
                for wh in self.alternatives.iter().filter_map(|x| x.as_ref()) {
                    wh.draw_warehouse(drawing);
                }
                for floating in self.floating_paths.iter() {
                    floating.draw_robot_path(
                        self.robot_in_improvement,
                        0.0,
                        Some(lookahead_turn + 1),
                        drawing,
                    );
                    floating.draw_robot(self.robot_in_improvement, 0.0, drawing);
                }
            }
            StageId::AddRollout => {
                let alpha = (self.stage.time / self.stage.id.get_duration().unwrap()).min(1.0);
                self.draw(&Stage::get_final(StageId::GenerateLookaheads), drawing);
                for wh in self.alternatives.iter().filter_map(|x| x.as_ref()) {
                    let max_len = wh
                        .get_paths()
                        .iter()
                        .map(|p| p.get_path_iterator().count())
                        .max()
                        .unwrap()
                        - lookahead_turn;
                    let current_len = (max_len as f32 * alpha).ceil() as usize + lookahead_turn;
                    wh.draw_warehouse(drawing);
                    for robot_index in 0..wh.get_paths().len() {
                        wh.draw_robot_path(robot_index, 0.0, Some(current_len), drawing);
                        wh.draw_robot(robot_index, 0.0, drawing);
                    }
                }
            }
            _ => {}
        }
    }

    fn stage_is_finished(&self) -> bool {
        use StageId::*;
        match self.stage.id {
            InitialWait => self.stage.time >= self.stage.id.get_duration().unwrap(),
            TranslateCurrentPath => self.stage.time >= self.stage.id.get_duration().unwrap(),
            GenerateLookaheads => self.stage.time >= self.stage.id.get_duration().unwrap(),
            CreateWarehouses => self.stage.time >= self.stage.id.get_duration().unwrap(),
            AddRollout => self.stage.time >= self.stage.id.get_duration().unwrap(),
            Simulate => self
                .alternatives
                .iter()
                .filter_map(|x| x.as_ref())
                .all(|w| w.is_finished()),
            PickBest => self.stage.time >= self.stage.id.get_duration().unwrap(),
        }
    }

    fn move_to_next_stage(&mut self) {
        use StageId::*;
        match self.stage.id {
            InitialWait => {
                self.generate_alternatives();
                self.floating_paths = self
                    .alternatives
                    .iter()
                    .filter_map(|x| x.as_ref())
                    .cloned()
                    .collect();
                self.stage.id = TranslateCurrentPath;
                self.stage.time = 0.0;
            }
            TranslateCurrentPath => {
                self.stage.id = GenerateLookaheads;
                self.stage.time = 0.0;
            }
            GenerateLookaheads => {
                self.stage.id = CreateWarehouses;
                self.stage.time = 0.0;
            }
            CreateWarehouses => {
                self.stage.id = AddRollout;
                self.stage.time = 0.0;
            }
            AddRollout => {
                self.stage.id = Simulate;
                self.stage.time = 0.0;
            }
            Simulate => {
                self.stage.id = PickBest;
                self.stage.time = 0.0;
            }
            PickBest => {
                self.stage.id = InitialWait;
                self.stage.time = 0.0;
            }
        }
    }

    fn update(&mut self, delta_time: f32) {
        self.stage.time += delta_time;
        println!("Stage time: {}", self.stage.time);

        match self.stage.id {
            StageId::InitialWait => {}
            StageId::TranslateCurrentPath => {
                let alpha = (self.stage.time / self.stage.id.get_duration().unwrap()).min(1.0);
                self.alternative_paths
                    .iter()
                    .filter_map(|x| x.as_ref())
                    .zip(self.floating_paths.iter_mut())
                    .for_each(|(alternative, floating)| {
                        let initial_position = self.base_waresim.get_location();
                        let final_position = alternative.get_location();
                        floating.set_location(initial_position.interpolate(final_position, alpha));
                    })
            }
            StageId::CreateWarehouses => {
                let alpha = (self.stage.time / self.stage.id.get_duration().unwrap()).min(1.0);
                self.alternative_paths
                    .iter()
                    .filter_map(|x| x.as_ref())
                    .zip(self.alternatives.iter().filter_map(|x| x.as_ref()))
                    .enumerate()
                    .for_each(|(i, (lookahead_show, rollout_wh))| {
                        let initial_position = lookahead_show.get_location();
                        let final_position = rollout_wh.get_location();
                        self.floating_paths[i]
                            .set_location(initial_position.interpolate(final_position, alpha));
                    })
            }
            _ => {}
        }

        if self.stage_is_finished() {
            self.move_to_next_stage();
        }
    }
}

fn initialize_model(app: &App) -> Model {
    app.set_loop_mode(nannou::app::LoopMode::wait());

    let mut walls = HashSet::new();
    walls.insert((0, 3));
    walls.insert((1, 3));
    walls.insert((3, 3));

    let endpoints = [((0, 0), (4, 5)), ((2, 7), (7, 3)), ((9, 7), (2, 3))];

    let warehouse = Warehouse::new((12, 8), walls);
    let paths = algorithm::initialize_paths(&warehouse, &endpoints);

    let drawhouse = WarehouseSim::new(
        warehouse.clone(),
        WareSimLocation::new((X_POS[0], Y_POS[2]), 20.0),
        paths,
        DEFAULT_SPEED,
    );

    let mut model = Model {
        warehouse,
        base_cost: 0,
        base_waresim: drawhouse,
        alternatives: vec![],
        alternative_paths: vec![],
        floating_paths: vec![],
        robot_in_improvement: 0,
        wait_time: None,
        stage: Stage::get_start(StageId::InitialWait),
    };
    model.generate_alternatives();

    model
}

fn update(_app: &App, model: &mut Model, update: Update) {
    model.update(update.since_last.as_secs_f32());
}
/*
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
*/

fn view(app: &App, model: &Model, frame: Frame) {
    frame.clear(GRAY);
    let draw = app.draw();
    draw.background().color(GRAY);

    model.draw(&model.stage, &draw);

    /*
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
    */

    draw.to_frame(app, &frame).unwrap();
    // app.main_window()
    //     .capture_frame(&format!("frames/{:0>4}.png", frame.nth()));
}
