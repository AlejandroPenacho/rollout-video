mod algorithm;
mod warehouse;

use warehouse::{AgentPath, Warehouse, WarehouseSim};

use nannou::prelude::*;

use std::collections::HashSet;

fn main() {
    nannou::app(initialize_model)
        .update(update)
        .simple_window(view)
        .run();
}

struct Model {
    waresims: Vec<WarehouseSim>,
}

fn initialize_model(_app: &App) -> Model {
    let mut walls = HashSet::new();
    walls.insert((0, 3));
    walls.insert((1, 3));
    walls.insert((3, 3));

    let endpoints = [((0, 0), (5, 4)), ((4, 5), (1, 2)), ((2, 3), (7, 3))];

    let warehouse = Warehouse::new((8, 12), walls);
    let paths = algorithm::initialize_paths(&warehouse, &endpoints);

    let drawhouse = WarehouseSim::new(warehouse, (0.0, 0.0), 40.0, paths, 1.0);

    Model {
        waresims: vec![drawhouse],
    }
}

fn update(_app: &App, model: &mut Model, update: Update) {
    model
        .waresims
        .iter_mut()
        .for_each(|w| w.update_time(update.since_last.as_secs_f32()));
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(BLUE);

    model.waresims.iter().for_each(|w| w.draw(&draw));

    draw.to_frame(app, &frame).unwrap();
}
