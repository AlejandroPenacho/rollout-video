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
    walls.insert((0, 0));
    walls.insert((0, 3));
    walls.insert((1, 3));
    walls.insert((2, 3));
    walls.insert((3, 3));

    let path_1 = AgentPath {
        initial_node: (0, 0),
        lookahead: vec![(0, 1), (1, 1), (1, 3)],
        a_star: vec![(1, 4), (2, 4), (5, 4)],
    };

    let path_2 = AgentPath {
        initial_node: (4, 5),
        lookahead: vec![(5, 5), (5, 4), (5, 3)],
        a_star: vec![(5, 2), (4, 2), (3, 2), (2, 2), (1, 2)],
    };

    let path_3 = AgentPath {
        initial_node: (2, 3),
        lookahead: vec![],
        a_star: vec![(3, 3), (4, 3), (5, 3), (6, 3), (7, 3)],
    };

    let warehouse = Warehouse::new((8, 12), walls);
    let paths = vec![path_1, path_2, path_3];

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

type Node = (i32, i32);

type StdColour = nannou::color::rgb::Rgb<nannou::color::encoding::Srgb, u8>;

const WALL_COLOR: StdColour = BLACK;
const NODE_COLOR_1: StdColour = CORAL;
const NODE_COLOR_2: StdColour = CORNFLOWERBLUE;
const LOOKAHEAD_COLOUR: StdColour = GREEN;
const ASTAR_COLOUR: StdColour = RED;
const COLLISION_COST: i32 = 200;

struct Warehouse {
    size: (i32, i32),
    walls: HashSet<Node>,
}

struct WarehouseSim {
    warehouse: Warehouse,
    position: (f32, f32),
    cell_size: f32,
    paths: Vec<AgentPath>,
    run_time: f32,
    speed: f32,
    cumulative_cost: i32,
    bubbles: Vec<TextBubble>,
    collisions: Vec<Vec<bool>>,
}

struct AgentPath {
    initial_node: Node,
    lookahead: Vec<Node>,
    a_star: Vec<Node>,
}

struct TextBubble {
    text: String,
    color: (f32, f32, f32),
    size: u32,
    initial_position: (f32, f32),
    travel_lenght: f32,
    travel_time: f32,
    initial_time: f32,
}

impl TextBubble {
    fn movement_cost(position: (f32, f32), cell_size: f32, initial_time: f32) -> TextBubble {
        TextBubble {
            text: String::from("+1"),
            color: (1.0, 0.0, 0.0),
            size: 30,
            initial_position: (position.0, position.1 + cell_size * 0.8),
            travel_lenght: cell_size,
            travel_time: 3.0,
            initial_time,
        }
    }

    fn collision_cost(position: (f32, f32), cell_size: f32, initial_time: f32) -> TextBubble {
        TextBubble {
            text: format!("+{:?}", COLLISION_COST),
            color: (1.0, 0.0, 0.0),
            size: 40,
            initial_position: (position.0, position.1 + cell_size * 0.8),
            travel_lenght: cell_size,
            travel_time: 3.0,
            initial_time,
        }
    }

    fn draw(&self, run_time: f32, draw: &Draw) {
        let status: f32 = (run_time - self.initial_time) / self.travel_time;
        let position = (
            self.initial_position.0,
            self.initial_position.1 + self.travel_lenght * status,
        );
        draw.text(&self.text)
            .rgba(
                self.color.0,
                self.color.1,
                self.color.2,
                1.0 - status.min(1.0),
            )
            .font_size(self.size)
            .x_y(position.0, position.1);
    }
}

impl Warehouse {
    fn new(size: (i32, i32), walls: HashSet<Node>) -> Self {
        Warehouse { size, walls }
    }
}

impl WarehouseSim {
    fn new(
        warehouse: Warehouse,
        position: (f32, f32),
        cell_size: f32,
        paths: Vec<AgentPath>,
        speed: f32,
    ) -> Self {
        let n_robots = paths.len();
        WarehouseSim {
            warehouse,
            position,
            cell_size,
            paths,
            run_time: 0.0,
            speed,
            bubbles: Vec::new(),
            cumulative_cost: 0,
            collisions: vec![vec![false; n_robots]; n_robots],
        }
    }

    fn fnode_to_coord(&self, node: &(f32, f32)) -> (f32, f32) {
        let base_node = (
            self.position.0 - self.cell_size * (self.warehouse.size.0 as f32 / 2.0),
            self.position.1 - self.cell_size * (self.warehouse.size.1 as f32 / 2.0),
        );

        (
            base_node.0 + node.0 * self.cell_size,
            base_node.1 + node.1 * self.cell_size,
        )
    }

    fn inode_to_coord(&self, node: &(i32, i32)) -> (f32, f32) {
        self.fnode_to_coord(&(node.0 as f32, node.1 as f32))
    }

    fn get_robot_position(&self, robot_index: usize, time: f32) -> (f32, f32) {
        let path = &self.paths[robot_index];
        let current_turn = time * self.speed;

        // The complete path of the robot is the obatained chaining the 3 parts
        let mut complete_path = std::iter::once(&path.initial_node)
            .chain(path.lookahead.iter())
            .chain(path.a_star.iter());

        if current_turn.floor() as usize >= complete_path.clone().count() - 1 {
            // The robot has reached its final destination, so is stays there
            let inode = complete_path.last().unwrap();
            (inode.0 as f32, inode.1 as f32)
        } else {
            // If that is not the case, we interpolate the position
            let index = current_turn.floor() as usize;
            let alpha = current_turn - index as f32;
            let prev_node = complete_path.nth(index).unwrap();
            let next_node = complete_path.next().unwrap();

            (
                (prev_node.0 as f32) * (1.0 - alpha) + (next_node.0 as f32) * alpha,
                (prev_node.1 as f32) * (1.0 - alpha) + (next_node.1 as f32) * alpha,
            )
        }
    }

    fn update_time(&mut self, delta_time: f32) {
        let new_time = self.run_time + delta_time;

        for robot_index in 0..self.paths.len() {
            let current_position = self.get_robot_position(robot_index, self.run_time);
            let new_position = self.get_robot_position(robot_index, new_time);

            let current_node = (
                current_position.0.round() as usize,
                current_position.1.round() as usize,
            );
            let new_node = (
                new_position.0.round() as usize,
                new_position.1.round() as usize,
            );
            if current_node.0 != new_node.0 || current_node.1 != new_node.1 {
                self.cumulative_cost += 1;
                self.bubbles.push(TextBubble::movement_cost(
                    self.fnode_to_coord(&new_position),
                    self.cell_size,
                    new_time,
                ));
            }
        }

        for robot_1 in 0..self.paths.len() {
            for robot_2 in (robot_1 + 1)..self.paths.len() {
                let pos_1 = self.get_robot_position(robot_1, new_time);
                let pos_2 = self.get_robot_position(robot_2, new_time);
                if (pos_1.0 - pos_2.0).powf(2.0) + (pos_1.1 - pos_2.1).powf(2.0) > (0.7).powf(2.0) {
                    self.collisions[robot_1][robot_2] = false;
                    self.collisions[robot_2][robot_1] = false;
                    continue;
                }

                if !self.collisions[robot_1][robot_2] {
                    self.cumulative_cost += COLLISION_COST;
                    let mean_position = ((pos_1.0 + pos_2.0) / 2.0, (pos_1.1 + pos_2.1) / 2.0);
                    self.bubbles.push(TextBubble::collision_cost(
                        self.fnode_to_coord(&mean_position),
                        self.cell_size,
                        new_time,
                    ));
                }
                self.collisions[robot_1][robot_2] = true;
                self.collisions[robot_2][robot_1] = true;
            }
        }

        let mut prev_bubbles = vec![];
        std::mem::swap(&mut self.bubbles, &mut prev_bubbles);
        self.bubbles = prev_bubbles
            .into_iter()
            .filter(|b| new_time - b.initial_time < b.travel_time)
            .collect();

        self.run_time += delta_time;
    }

    fn draw(&self, drawing: &nannou::draw::Draw) {
        self.draw_warehouse(drawing);
        for robot_index in 0..self.paths.len() {
            self.draw_robot_path(robot_index, self.run_time, drawing);
        }
        // drawing.text("Compramos tu coche").font_size(50).w(100.0);
        self.bubbles
            .iter()
            .for_each(|b| b.draw(self.run_time, drawing));
    }

    fn draw_warehouse(&self, drawing: &nannou::draw::Draw) {
        for i in 0..self.warehouse.size.1 {
            for j in 0..self.warehouse.size.0 {
                let (x, y) = self.inode_to_coord(&(i, j));
                let color = if self.warehouse.walls.contains(&(i, j)) {
                    WALL_COLOR
                } else if (i + j) % 2 == 0 {
                    NODE_COLOR_1
                } else {
                    NODE_COLOR_2
                };
                drawing
                    .rect()
                    .color(color)
                    .x_y(x, y)
                    .w(self.cell_size)
                    .h(self.cell_size);
            }
        }
    }

    fn draw_robot_path(&self, robot_index: usize, time: f32, drawing: &nannou::draw::Draw) {
        let diameter = 0.7 * self.cell_size;

        // The current turn, which is the point in the path in which the robot is,
        // starts at 0 at start_time and increses by "speed" each second.
        let current_turn = time * self.speed;

        let path = &self.paths[robot_index];

        let current_node = self.get_robot_position(robot_index, time);
        let mut current_node_in_path = current_node;

        let path_iterator = path
            .lookahead
            .iter()
            .map(|n| (n, LOOKAHEAD_COLOUR))
            .chain(path.a_star.iter().map(|n| (n, ASTAR_COLOUR)));

        for (node, colour) in path_iterator.skip(current_turn.floor() as usize) {
            let start = self.fnode_to_coord(&current_node_in_path);
            let end = self.inode_to_coord(node);

            drawing
                .line()
                .start(pt2(start.0, start.1))
                .end(pt2(end.0, end.1))
                .weight(self.cell_size / 8.0)
                .color(colour);

            current_node_in_path = (node.0 as f32, node.1 as f32);
        }

        let position = self.fnode_to_coord(&current_node);
        let colour = if self.collisions[robot_index].iter().any(|&x| x) {
            ORANGE
        } else {
            GREEN
        };

        drawing
            .ellipse()
            .color(colour)
            .x_y(position.0, position.1)
            .w_h(diameter, diameter);
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(BLUE);

    model.waresims.iter().for_each(|w| w.draw(&draw));

    draw.to_frame(app, &frame).unwrap();
}
