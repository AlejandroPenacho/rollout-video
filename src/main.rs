use nannou::prelude::*;

use std::collections::HashSet;

fn main() {
    nannou::app(|_| Model {})
        .update(update)
        .simple_window(view)
        .run();
}

struct Model {}

fn update(_app: &App, _model: &mut Model, _update: Update) {}

type Node = (i32, i32);

type StdColour = nannou::color::rgb::Rgb<nannou::color::encoding::Srgb, u8>;

const WALL_COLOR: StdColour = BLACK;
const NODE_COLOR_1: StdColour = CORAL;
const NODE_COLOR_2: StdColour = CORNFLOWERBLUE;
const LOOKAHEAD_COLOUR: StdColour = GREEN;
const ASTAR_COLOUR: StdColour = RED;

struct Warehouse {
    size: (i32, i32),
    walls: HashSet<Node>,
}

struct WarehouseSim<'a> {
    warehouse: &'a Warehouse,
    position: (f32, f32),
    cell_size: f32,
    paths: Vec<AgentPath>,
    start_time: f32,
    speed: f32,
}

struct AgentPath {
    initial_node: Node,
    lookahead: Vec<Node>,
    a_star: Vec<Node>,
}

impl Warehouse {
    fn new(size: (i32, i32), walls: HashSet<Node>) -> Self {
        Warehouse { size, walls }
    }
}

impl<'a> WarehouseSim<'a> {
    fn new(
        warehouse: &'a Warehouse,
        position: (f32, f32),
        cell_size: f32,
        paths: Vec<AgentPath>,
        app: &App,
        speed: f32,
    ) -> Self {
        WarehouseSim {
            warehouse,
            position,
            cell_size,
            paths,
            start_time: app.time,
            speed,
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

    fn draw(&self, drawing: &nannou::draw::Draw, app: &App) {
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

        for path in self.paths.iter() {
            self.draw_robot_path(path, app.time, drawing);
        }
    }

    fn draw_robot_path(&self, path: &AgentPath, time: f32, drawing: &nannou::draw::Draw) {
        let diameter = self.cell_size;

        // The current turn, which is the point in the path in which the robot is,
        // starts at 0 at start_time and increses by "speed" each second.
        let current_turn = (time/* - self.start_time */) * self.speed;

        // The complete path of the robot is the obatained chaining the 3 parts
        let mut complete_path = std::iter::once(&path.initial_node)
            .chain(path.lookahead.iter())
            .chain(path.a_star.iter());

        let current_node: (f32, f32) =
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
            };

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

        drawing
            .ellipse()
            .x_y(position.0, position.1)
            .w_h(diameter, diameter);
    }
}

fn view(app: &App, _model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(BLUE);

    let mut walls = HashSet::new();
    walls.insert((0, 0));
    walls.insert((0, 3));
    walls.insert((1, 3));
    walls.insert((2, 3));
    walls.insert((3, 3));

    let path = AgentPath {
        initial_node: (0, 0),
        lookahead: vec![(0, 1), (1, 1), (1, 3)],
        a_star: vec![(1, 4), (2, 4), (5, 4)],
    };

    let warehouse = Warehouse::new((8, 12), walls);
    let paths = vec![path];

    let drawhouse = WarehouseSim::new(&warehouse, (0.0, 0.0), 40.0, paths, app, 1.0);

    drawhouse.draw(&draw, app);

    let time = app.time;

    let horizontal = time.sin();
    let vertical = (time * 2.0).sin();

    let boundary = app.window_rect();
    let x = map_range(horizontal, -1.0, 1.0, boundary.left(), boundary.right());
    let y = map_range(vertical, -1.0, 1.0, boundary.bottom(), boundary.top());

    draw.ellipse().color(RED).x_y(x, y);
    draw.to_frame(app, &frame).unwrap();
}
