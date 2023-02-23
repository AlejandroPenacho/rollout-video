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
    best_alternative: usize,
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

fn get_best_policy_color() -> warehouse::StdColour {
    warehouse::StdColour::new(0, 86, 0)
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

    fn get_best_policy(&self) -> usize {
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

        if self.alternatives[best_index]
            .as_ref()
            .unwrap()
            .get_current_cost()
            == self.alternatives[no_change_index]
                .as_ref()
                .unwrap()
                .get_current_cost()
        {
            best_index = no_change_index;
        }

        best_index

        // let mut best_policy = self.alternatives[best_index]
        //     .as_ref()
        //     .unwrap()
        //     .get_paths()
        //     .to_owned();
        // best_policy.iter_mut().for_each(|p| p.integrate_lookahead());

        // self.base_cost = improved_cost;

        // self.base_waresim = WarehouseSim::new(
        //     self.warehouse.clone(),
        //     WareSimLocation::new((X_POS[0], Y_POS[2]), 20.0),
        //     best_policy,
        //     DEFAULT_SPEED,
        // );

        // self.robot_in_improvement =
        //     (self.robot_in_improvement + 1) % self.base_waresim.get_paths().len();
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

                use warehouse::{get_colour, ColorCollection};
                let base_position = (X_POS[0], Y_POS[4]);
                let agent_diameter = 0.7 * self.base_waresim.get_location().cell_size;
                for agent_index in 0..self.base_waresim.get_paths().len() {
                    let y_pos = base_position.1 - agent_index as f32 * 50.0;

                    if agent_index == self.robot_in_improvement {
                        drawing
                            .rect()
                            .x_y(base_position.0 - 50.0, y_pos)
                            .w_h(150.0, 50.0)
                            .color(GREEN);
                    };

                    drawing
                        .text(&format!("Agent {}", agent_index + 1))
                        .x_y(base_position.0 - 100.0, y_pos);
                    drawing
                        .ellipse()
                        .x_y(base_position.0, y_pos)
                        .color(get_colour(ColorCollection::DARK, agent_index))
                        .w_h(agent_diameter, agent_diameter);
                }
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
                    for i in 0..wh.get_paths().len() {
                        wh.draw_robot(i, 0.0, drawing);
                    }
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
            StageId::Simulate => {
                self.base_waresim.draw(drawing);
                self.draw(&Stage::get_final(StageId::GenerateLookaheads), drawing);
                self.alternatives
                    .iter()
                    .enumerate()
                    .filter_map(|(i, x)| x.as_ref().map(|x| (i, x)))
                    .for_each(|(i, w)| {
                        w.draw(drawing);
                        w.draw_cost((X_POS[3], Y_POS[i]), drawing);
                    });
            }
            StageId::PickBest => {
                drawing
                    .rect()
                    .x_y((X_POS[1] + X_POS[3]) / 2.0, Y_POS[self.best_alternative])
                    .w_h(600.0, 200.0)
                    .color(get_best_policy_color());
                self.draw(&Stage::get_final(StageId::Simulate), drawing);
                let floater = &self.floating_paths[0];

                floater.draw_robot_path(
                    self.robot_in_improvement,
                    0.0,
                    Some(lookahead_turn + 1),
                    drawing,
                );
                floater.draw_robot(self.robot_in_improvement, 0.0, drawing);
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
                    .filter_map(|x| {
                        let mut x = x.as_ref()?.clone();
                        x.set_location(self.base_waresim.get_location().clone());
                        Some(x)
                    })
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
                self.alternatives.iter_mut().for_each(|x| {
                    if let Some(x) = x {
                        x.toggle_running(true);
                    }
                });
                self.stage.id = Simulate;
                self.stage.time = 0.0;
            }
            Simulate => {
                self.best_alternative = self.get_best_policy();
                self.alternative_paths[self.best_alternative]
                    .as_mut()
                    .unwrap()
                    .get_mut_paths()
                    .iter_mut()
                    .for_each(|p| p.integrate_lookahead());

                self.floating_paths = vec![self.alternative_paths[self.best_alternative]
                    .clone()
                    .unwrap()];

                self.stage.id = PickBest;
                self.stage.time = 0.0;
            }
            PickBest => {
                self.base_waresim = WarehouseSim::new(
                    self.warehouse.clone(),
                    WareSimLocation::new((X_POS[0], Y_POS[2]), 20.0),
                    self.alternative_paths[self.best_alternative]
                        .as_ref()
                        .unwrap()
                        .get_paths()
                        .to_owned(),
                    DEFAULT_SPEED,
                );

                self.base_cost = self.alternatives[self.best_alternative]
                    .as_ref()
                    .unwrap()
                    .get_current_cost();
                self.robot_in_improvement =
                    (self.robot_in_improvement + 1) % self.base_waresim.get_paths().len();
                self.stage.id = InitialWait;
                self.stage.time = 0.0;
            }
        }
    }

    fn update(&mut self, delta_time: f32) {
        self.stage.time += delta_time;

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
            StageId::Simulate => {
                self.alternatives
                    .iter_mut()
                    .filter_map(|x| x.as_mut())
                    .for_each(|w| w.update_time(delta_time));
            }
            StageId::PickBest => {
                let alpha = (self.stage.time / self.stage.id.get_duration().unwrap()).min(1.0);
                self.floating_paths[0].set_location(
                    self.alternative_paths[self.best_alternative]
                        .as_mut()
                        .unwrap()
                        .get_location()
                        .interpolate(self.base_waresim.get_location(), alpha),
                );
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
    // walls.insert((0, 3));
    // walls.insert((1, 3));
    // walls.insert((3, 3));

    // let endpoints = [((0, 0), (4, 5)), ((2, 7), (7, 3)), ((9, 7), (2, 3))];

    let endpoints = [((4, 4), (8, 4)), ((5, 4), (1, 4)), ((9, 7), (2, 3))];

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
        best_alternative: 0,
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

fn view(app: &App, model: &Model, frame: Frame) {
    frame.clear(GRAY);
    let draw = app.draw();
    draw.background().color(GRAY);

    model.draw(&model.stage, &draw);

    draw.to_frame(app, &frame).unwrap();
    // app.main_window()
    //     .capture_frame(&format!("frames/{:0>4}.png", frame.nth()));
}
