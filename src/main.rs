use nannou::prelude::*;

use std::collections::HashSet;

fn main() {
    nannou::sketch(view).run();
}

type Node = (i32, i32);

struct Warehouse {
    size: (i32, i32),
    walls: HashSet<Node>,
    drawing: Option<(f32, f32, f32)>,
}

impl Warehouse {
    fn new(size: (i32, i32), walls: HashSet<Node>) -> Self {
        Warehouse {
            size,
            walls,
            drawing: None,
        }
    }

    fn draw(
        &mut self,
        topleft_position: (f32, f32),
        cell_size: f32,
        app: &App,
        drawing: &nannou::draw::Draw,
    ) {
        self.drawing = Some((topleft_position.0, topleft_position.1, cell_size));

        for i in 0..self.size.1 {
            for j in 0..self.size.0 {
                let x = topleft_position.0 + cell_size * (i as f32 + 0.5);
                let y = topleft_position.1 + cell_size * (j as f32 + 0.5);
                let color = if self.walls.contains(&(i, j)) {
                    BLACK
                } else if (i + j) % 2 == 0 {
                    CORAL
                } else {
                    CORNFLOWERBLUE
                };
                drawing
                    .rect()
                    .color(color)
                    .x_y(x, y)
                    .w(cell_size)
                    .h(cell_size);
            }
        }
    }

    fn draw_robot(&self, path: &[Node], time: f32, drawing: &nannou::draw::Draw) {
        let diameter = self.drawing.unwrap().2;
        let current_node = if time >= (path.len() - 1) as f32 {
            let node = path.iter().last().unwrap();
            (node.0 as f32, node.1 as f32)
        } else {
            let index: usize = time.floor() as usize;
            let alpha: f32 = time - time.floor();
            (
                path[index].0 as f32 * (1.0 - alpha) + path[index + 1].0 as f32 * alpha,
                path[index].1 as f32 * (1.0 - alpha) + path[index + 1].1 as f32 * alpha,
            )
        };

        let position = (
            self.drawing.unwrap().0 + self.drawing.unwrap().2 * (current_node.0 + 0.5),
            self.drawing.unwrap().1 + self.drawing.unwrap().2 * (current_node.1 + 0.5),
        );

        drawing
            .ellipse()
            .x_y(position.0, position.1)
            .w_h(diameter, diameter);
    }
}

fn view(app: &App, frame: Frame) {
    let draw = app.draw();
    draw.background().color(BLUE);

    let mut walls = HashSet::new();
    walls.insert((0, 0));
    walls.insert((0, 3));
    walls.insert((1, 3));
    walls.insert((2, 3));
    walls.insert((3, 3));

    let path = vec![(0, 0), (0, 1), (1, 1), (1, 3), (1, 4), (2, 4), (5, 4)];

    let mut warehouse = Warehouse::new((8, 12), walls);
    warehouse.draw((-100.0, 0.0), 40.0, app, &draw);
    let time = app.time;
    warehouse.draw_robot(&path, time, &draw);

    let horizontal = time.sin();
    let vertical = (time * 2.0).sin();

    let boundary = app.window_rect();
    let x = map_range(horizontal, -1.0, 1.0, boundary.left(), boundary.right());
    let y = map_range(vertical, -1.0, 1.0, boundary.bottom(), boundary.top());

    draw.ellipse().color(RED).x_y(x, y);
    draw.to_frame(app, &frame).unwrap();
}
