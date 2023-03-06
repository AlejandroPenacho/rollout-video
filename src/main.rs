mod algorithm;
mod warehouse;

use warehouse::{WareSimLocation, Warehouse, WarehouseSim};

use nannou::prelude::*;

use std::collections::HashSet;

fn main() {
    nannou::app(|app| initialize_model(app, 0))
        .update(update)
        .simple_window(view)
        .run();

    // nannou::sketch(sketch_creator).run();
}

const DEFAULT_SPEED: f32 = 2.0;

const Y_POS: [f32; 5] = [-300.0, -150.0, 0.0, 150.0, 300.0];
const X_POS: [f32; 4] = [-200.0, 0.0, 200.0, 400.0];
const LOOKAHEAD_VISUAL_SIZE: f32 = 100.0;

const NEW_ORDER: [usize; 3] = [2, 1, 0];
const TITLES: [&str; 3] = ["Case 1", "Case 2", "Case 3"];

struct Model {
    wait_time: Option<f32>,
    warehouse: Warehouse,
    best_alternative: usize,
    base_cost: i32,
    original_policy_cost: i32,
    base_waresim: WarehouseSim,
    base_waresim_backup: WarehouseSim,
    alternatives: Vec<Option<WarehouseSim>>,
    alternative_paths: Vec<Option<WarehouseSim>>,
    floating_paths: Vec<WarehouseSim>,
    robot_in_improvement: usize,
    model_id: usize,
    stage: Stage,
    improvement_order: Vec<usize>,
}

#[derive(Clone, Copy)]
struct Stage {
    id: StageId,
    time: f32,
}

#[derive(Clone, Copy)]
enum StageId {
    Title,
    PreWait,
    InitialSimulation,
    InitialWait,
    TranslateCurrentPath,
    GenerateLookaheads,
    CreateWarehouses,
    AddRollout,
    Simulate,
    PickBest,
    Reshuffle,
    FinalWait,
}

impl StageId {
    fn get_duration(&self) -> Option<f32> {
        use StageId::*;
        match self {
            AddRollout => Some(3.0),
            GenerateLookaheads => Some(1.0),
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

const LOOKAHEAD_NAMES: [&str; 5] = ["Stay", "Go up", "Go right", "Go down", "Go left"];

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
    }
}

impl Model {
    fn draw(&self, stage: &Stage, drawing: &Draw) {
        let lookahead_turn = self.base_waresim.get_paths()[self.robot_in_improvement]
            .improved_path
            .len();

        match &stage.id {
            StageId::Title => {
                drawing.text(TITLES[self.model_id]).font_size(70);
            }
            StageId::PreWait => {
                self.draw(&Stage::get_start(StageId::InitialWait), drawing);
            }
            StageId::InitialSimulation => {
                self.draw(&Stage::get_start(StageId::InitialWait), drawing);
            }
            StageId::InitialWait => {
                self.base_waresim.draw(drawing);
                drawing
                    .text("Current policy")
                    .x_y(X_POS[0], 100.0)
                    .font_size(20);

                drawing.text("Cost:").x_y(X_POS[0], -75.0).font_size(20);

                drawing
                    .text(&format!("{}", self.base_cost))
                    .x_y(X_POS[0], -100.0)
                    .font_size(20);

                draw_order(
                    &self.improvement_order,
                    &self.improvement_order,
                    self.base_waresim.get_location().cell_size,
                    Some(self.robot_in_improvement),
                    0.0,
                    drawing,
                );
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
                drawing
                    .text("Simulations")
                    .x_y(X_POS[2], Y_POS[4] + 150.0)
                    .font_size(25);

                self.alternative_paths
                    .iter()
                    .enumerate()
                    .for_each(|(i, w)| {
                        drawing
                            .text(LOOKAHEAD_NAMES[i])
                            .font_size(20)
                            .x_y(X_POS[1], Y_POS[i] + LOOKAHEAD_VISUAL_SIZE / 2.0);

                        if let Some(w) = w {
                            w.draw_robot_path(
                                self.robot_in_improvement,
                                0.0,
                                Some(lookahead_turn + 1),
                                drawing,
                            );
                            w.draw_robot(self.robot_in_improvement, 0.0, drawing);
                        } else {
                            drawing.text("Infeasible").x_y(X_POS[1], Y_POS[i]);
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
                let alpha_1 =
                    (2.0 * self.stage.time / self.stage.id.get_duration().unwrap()).min(1.0);
                self.draw(&Stage::get_final(StageId::GenerateLookaheads), drawing);
                for wh in self.alternatives.iter().filter_map(|x| x.as_ref()) {
                    let max_len = wh
                        .get_paths()
                        .iter()
                        .map(|p| p.get_path_iterator().count())
                        .max()
                        .unwrap()
                        - lookahead_turn;
                    let current_len =
                        (max_len as f32 * alpha_1).ceil() as usize + lookahead_turn + 1;
                    wh.draw_warehouse(drawing);
                    for robot_index in 0..wh.get_paths().len() {
                        if robot_index == self.robot_in_improvement {
                            wh.draw_robot_path(robot_index, 0.0, Some(current_len), drawing);
                        }
                        wh.draw_robot(robot_index, 0.0, drawing);
                    }
                }

                for floating in self.floating_paths.iter() {
                    for robot_index in (0..self.base_waresim.get_paths().len())
                        .filter(|&x| x != self.robot_in_improvement)
                    {
                        // floating.draw_robot(robot_index, 0.0, drawing);
                        floating.draw_robot_path(robot_index, 0.0, None, drawing);
                    }
                }
                for robot_index in 0..self.base_waresim.get_paths().len() {
                    self.base_waresim.draw_robot(robot_index, 0.0, drawing);
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
                    .w_h(600.0, 150.0)
                    .color(warehouse::get_colour(warehouse::ColorCollection::DARK, 3));
                self.draw(&Stage::get_final(StageId::Simulate), drawing);
                let floater = &self.floating_paths[0];

                floater.draw_robot_path(self.robot_in_improvement, 0.0, None, drawing);
            }
            StageId::Reshuffle => {
                let alpha = (self.stage.time / self.stage.id.get_duration().unwrap()).min(1.0);
                let alpha = -2.0 * alpha.powf(3.0) + 3.0 * alpha.powf(2.0);
                drawing
                    .text("Current policy")
                    .x_y(X_POS[0], 100.0)
                    .font_size(20);
                self.base_waresim.draw(drawing);
                draw_order(
                    &self.improvement_order,
                    &NEW_ORDER,
                    self.base_waresim.get_location().cell_size,
                    None,
                    alpha,
                    drawing,
                );
                drawing
                    .text("Reshuffling")
                    .font_size(50)
                    .x_y(X_POS[1], Y_POS[4] - 50.0)
                    .width(400.0);
                drawing
                    .text("Previous policy is restored")
                    .font_size(15)
                    .x_y(X_POS[0], -100.0);
            }
            StageId::FinalWait => {
                self.draw(&Stage::get_final(StageId::InitialWait), drawing);
            }
        }
    }

    fn stage_is_finished(&self) -> bool {
        use StageId::*;
        match self.stage.id {
            Title => self.stage.time >= self.stage.id.get_duration().unwrap(),
            PreWait => self.stage.time >= self.stage.id.get_duration().unwrap(),
            InitialSimulation => self.base_waresim.is_finished(),
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
            Reshuffle => self.stage.time >= self.stage.id.get_duration().unwrap(),
            FinalWait => self.stage.time >= self.stage.id.get_duration().unwrap(),
        }
    }

    fn move_to_next_stage(&mut self, app: &App) {
        use StageId::*;
        match self.stage.id {
            Title => {
                self.stage.id = PreWait;
                self.stage.time = 0.0;
            }
            PreWait => {
                self.stage.id = InitialSimulation;
                self.base_waresim.toggle_running(true);
                self.stage.time = 0.0;
            }
            InitialSimulation => {
                self.base_waresim = self.base_waresim_backup.clone();
                self.original_policy_cost = self.base_cost;
                self.stage.id = InitialWait;
                self.stage.time = 0.0;
            }
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
                self.floating_paths = self
                    .alternatives
                    .iter()
                    .filter_map(|x| x.as_ref())
                    .map(|_| self.base_waresim.clone())
                    .collect();
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

                let mut best_waresim = self.alternatives[self.best_alternative]
                    .as_ref()
                    .unwrap()
                    .clone();
                best_waresim
                    .get_mut_paths()
                    .iter_mut()
                    .for_each(|p| p.integrate_lookahead());

                self.floating_paths = vec![WarehouseSim::new(
                    best_waresim.get_warehouse().clone(),
                    best_waresim.get_location().clone(),
                    best_waresim.get_paths().to_owned(),
                    DEFAULT_SPEED,
                )];

                self.stage.id = PickBest;
                self.stage.time = 0.0;
            }
            PickBest => {
                self.base_waresim = WarehouseSim::new(
                    self.warehouse.clone(),
                    WareSimLocation::new((X_POS[0], 0.0), 20.0),
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

                let current_index = self
                    .improvement_order
                    .iter()
                    .position(|&x| x == self.robot_in_improvement)
                    .unwrap();

                let new_index = (current_index + 1) % self.improvement_order.len();

                if new_index != 0 {
                    self.robot_in_improvement = self.improvement_order[new_index];
                    self.stage.id = InitialWait;
                    self.stage.time = 0.0;
                    return;
                }

                if self.base_cost > warehouse::COLLISION_COST {
                    self.stage.id = Reshuffle;
                    self.stage.time = 0.0;
                    self.base_waresim = self.base_waresim_backup.clone();
                } else {
                    self.stage.id = FinalWait;
                    self.stage.time = 0.0;
                }
            }
            StageId::Reshuffle => {
                self.improvement_order = NEW_ORDER.to_vec();
                self.robot_in_improvement = self.improvement_order[0];

                self.base_cost = self.original_policy_cost;

                self.stage.id = InitialWait;
                self.stage.time = 0.0;
            }

            StageId::FinalWait => {
                let next_model = self.model_id + 1;
                if next_model == 3 {
                    app.quit()
                } else {
                    *self = initialize_model(app, next_model);
                }
            }
        }
    }

    fn update(&mut self, delta_time: f32, app: &App) {
        self.stage.time += delta_time;

        match self.stage.id {
            StageId::InitialSimulation => {
                self.base_waresim.update_time(delta_time);
                self.base_cost = self.base_waresim.get_current_cost();
            }
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
            StageId::AddRollout => {
                let alpha_2 = (2.0 * self.stage.time / self.stage.id.get_duration().unwrap() - 1.0)
                    .min(1.0)
                    .max(0.0);
                self.floating_paths
                    .iter_mut()
                    .zip(self.alternatives.iter().filter_map(|x| x.as_ref()))
                    .for_each(|(floating, dest)| {
                        let initial_loc = self.base_waresim.get_location();
                        let final_loc = dest.get_location();
                        floating.set_location(initial_loc.interpolate(final_loc, alpha_2));
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
                    self.alternatives[self.best_alternative]
                        .as_mut()
                        .unwrap()
                        .get_location()
                        .interpolate(self.base_waresim.get_location(), alpha),
                );
            }
            StageId::Reshuffle => {}
            _ => {}
        }

        if self.stage_is_finished() {
            self.move_to_next_stage(app);
        }
    }
}

fn draw_order(
    old_order: &[usize],
    new_order: &[usize],
    cell_size: f32,
    robot_in_improvement: Option<usize>,
    alpha: f32,
    drawing: &Draw,
) {
    use warehouse::{get_colour, ColorCollection};

    let base_position = (X_POS[0], Y_POS[4]);
    let agent_diameter = 0.7 * cell_size;
    for agent_index in 0..old_order.len() {
        let old_pos_index = old_order.iter().position(|&x| x == agent_index).unwrap();
        let new_pos_index = new_order.iter().position(|&x| x == agent_index).unwrap();

        let alpha_index = old_pos_index as f32 * (1.0 - alpha) + new_pos_index as f32 * alpha;

        let y_pos = base_position.1 - alpha_index * 50.0;

        if robot_in_improvement.map_or(false, |x| x == agent_index) {
            drawing
                .rect()
                .x_y(base_position.0 - 30.0, y_pos)
                .w_h(110.0, 50.0)
                .color(get_colour(ColorCollection::DARK, 3));
        };

        drawing
            .text(&format!("Agent {}", agent_index + 1))
            .x_y(base_position.0 - 50.0, y_pos);
        drawing
            .ellipse()
            .x_y(base_position.0, y_pos)
            .color(get_colour(ColorCollection::DARK, agent_index))
            .w_h(agent_diameter, agent_diameter);
    }
    drawing
        .text("Improvement order")
        .rotate(std::f32::consts::FRAC_PI_2)
        .x_y(base_position.0 - 130.0, base_position.1 - 50.0)
        .font_size(18);
    drawing
        .line()
        .start(pt2(base_position.0 - 100.0, base_position.1 + 30.0))
        .end(pt2(base_position.0 - 100.0, base_position.1 - 130.0))
        .weight(3.0)
        .color(WHITE);
    drawing
        .tri()
        .x_y(base_position.0 - 100.0, base_position.1 - 140.0)
        .rotate(-std::f32::consts::FRAC_PI_2)
        .w_h(20.0, 20.0);
}

fn initialize_model(app: &App, model_id: usize) -> Model {
    app.set_loop_mode(nannou::app::LoopMode::wait());

    let mut walls = HashSet::new();
    walls.insert((2, 2));
    walls.insert((3, 2));
    walls.insert((5, 0));

    let warehouse = Warehouse::new((8, 4), walls);

    let drawhouse: WarehouseSim = match model_id {
        0 => {
            let endpoints = [((1, 1), (6, 0)), ((3, 0), (4, 3)), ((6, 3), (2, 3))];
            let paths = algorithm::initialize_paths(&warehouse, &endpoints);

            WarehouseSim::new(
                warehouse,
                WareSimLocation::new((X_POS[0], 0.0), 20.0),
                paths,
                DEFAULT_SPEED,
            )
        }
        1 => {
            let endpoints = [((1, 1), (5, 3)), ((2, 0), (7, 1)), ((6, 2), (4, 0))];
            let paths = algorithm::initialize_paths(&warehouse, &endpoints);

            WarehouseSim::new(
                warehouse,
                WareSimLocation::new((X_POS[0], 0.0), 20.0),
                paths,
                DEFAULT_SPEED,
            )
        }
        2 => {
            let endpoints = [((2, 3), (7, 3)), ((1, 3), (6, 1)), ((5, 3), (0, 1))];
            let paths = algorithm::initialize_paths(&warehouse, &endpoints);

            WarehouseSim::new(
                warehouse,
                WareSimLocation::new((X_POS[0], 0.0), 20.0),
                paths,
                DEFAULT_SPEED,
            )
        }
        _ => {
            panic!()
        }
    };

    let mut model = Model {
        warehouse: drawhouse.get_warehouse().clone(),
        base_cost: 0,
        original_policy_cost: 0,
        base_waresim: drawhouse.clone(),
        base_waresim_backup: drawhouse,
        best_alternative: 0,
        alternatives: vec![],
        alternative_paths: vec![],
        floating_paths: vec![],
        robot_in_improvement: 0,
        wait_time: None,
        model_id,
        stage: Stage::get_start(StageId::Title),
        improvement_order: vec![0, 1, 2],
    };
    model.generate_alternatives();

    model
}

fn update(app: &App, model: &mut Model, update: Update) {
    model.update(update.since_last.as_secs_f32(), app);
}

fn view(app: &App, model: &Model, frame: Frame) {
    frame.clear(GRAY);
    let draw = app.draw();
    draw.background().color(GRAY);

    model.draw(&model.stage, &draw);

    draw.to_frame(app, &frame).unwrap();
    app.main_window()
        .capture_frame(&format!("frames/{:0>5}.png", frame.nth()));
}

fn sketch_creator(app: &App, frame: Frame) {
    let draw = app.draw();
    draw.background().color(WHITE);

    let mut warehouse_0 = initialize_model(app, 0).base_waresim;
    let mut warehouse_1 = initialize_model(app, 1).base_waresim;
    let mut warehouse_2 = initialize_model(app, 2).base_waresim;

    let y_pos = [300.0, 0.0, -300.0];

    warehouse_0.set_location(WareSimLocation {
        position: (0.0, y_pos[0]),
        cell_size: 55.0,
    });
    warehouse_1.set_location(WareSimLocation {
        position: (0.0, y_pos[1]),
        cell_size: 55.0,
    });
    warehouse_2.set_location(WareSimLocation {
        position: (0.0, y_pos[2]),
        cell_size: 55.0,
    });

    warehouse_0.draw_warehouse(&draw);
    warehouse_1.draw_warehouse(&draw);
    warehouse_2.draw_warehouse(&draw);

    for robot_index in 0..3 {
        warehouse_0.draw_robot_path(robot_index, 0.0, None, &draw);
        warehouse_1.draw_robot_path(robot_index, 0.0, None, &draw);
        warehouse_2.draw_robot_path(robot_index, 0.0, None, &draw);
    }
    for robot_index in 0..3 {
        warehouse_0.draw_robot(robot_index, 0.0, &draw);
        warehouse_1.draw_robot(robot_index, 0.0, &draw);
        warehouse_2.draw_robot(robot_index, 0.0, &draw);
    }

    app.main_window().capture_frame("example.png");
    draw.to_frame(app, &frame).unwrap();
}
