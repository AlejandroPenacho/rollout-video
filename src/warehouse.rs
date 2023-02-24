use nannou::prelude::*;

use std::collections::HashSet;

#[macro_export]
macro_rules! walls {
    () => {{
        std::collections::HashSet::new()
    }};
    ($($x:expr),*) => {{
        let mut walls = std::collections::HashSet::new();
        $(
            walls.insert($x);
        )*
        walls
    }}
}

pub type Node = (i32, i32);

pub type StdColour = nannou::color::rgb::Rgb<nannou::color::encoding::Srgb, u8>;

pub enum ColorCollection {
    DARK,
    LIGHT,
    GRAY,
}

pub fn get_colour(collection: ColorCollection, index: usize) -> StdColour {
    use ColorCollection::*;
    match collection {
        DARK => [
            StdColour::new(0, 45, 90),
            StdColour::new(154, 9, 50),
            StdColour::new(64, 102, 24),
            StdColour::new(235, 105, 0),
        ][index],
        LIGHT => [
            StdColour::new(0, 174, 239),
            StdColour::new(225, 5, 109),
            StdColour::new(102, 190, 0),
            StdColour::new(255, 212, 0),
        ][index],
        GRAY => [
            StdColour::new(224, 222, 217),
            StdColour::new(167, 157, 150),
            StdColour::new(104, 92, 83),
        ][index],
    }
}

const LOOKAHEAD_COLOUR: StdColour = GREEN;
pub const COLLISION_COST: i32 = 200;

#[derive(Clone)]
pub struct Warehouse {
    size: (i32, i32),
    walls: HashSet<Node>,
}

#[derive(Clone)]
pub struct WareSimLocation {
    pub position: (f32, f32),
    pub cell_size: f32,
}

impl WareSimLocation {
    pub fn new(position: (f32, f32), cell_size: f32) -> Self {
        Self {
            position,
            cell_size,
        }
    }

    pub fn interpolate(&self, other: &Self, alpha: f32) -> Self {
        let alpha = -2.0 * alpha.powf(3.0) + 3.0 * alpha.powf(2.0);
        Self {
            position: (
                self.position.0 * (1.0 - alpha) + other.position.0 * alpha,
                self.position.1 * (1.0 - alpha) + other.position.1 * alpha,
            ),
            cell_size: self.cell_size * (1.0 - alpha) + other.cell_size * alpha,
        }
    }
}

#[derive(Clone)]
pub struct WarehouseSim {
    warehouse: Warehouse,
    location: WareSimLocation,
    paths: Vec<AgentPath>,
    drawn_paths: Vec<Vec<((f32, f32), StdColour)>>,
    running: bool,
    run_time: f32,
    speed: f32,
    cumulative_cost: i32,
    bubbles: Vec<TextBubble>,
    collisions: Vec<Vec<bool>>,
}

#[derive(Clone, Debug)]
pub struct AgentPath {
    pub improved_path: Vec<Node>,
    pub lookahead: Vec<Node>,
    pub a_star: Vec<Node>,
}

impl AgentPath {
    pub fn new(improved_path: Vec<Node>, lookahead: Vec<Node>, a_star: Vec<Node>) -> Self {
        AgentPath {
            improved_path,
            lookahead,
            a_star,
        }
    }

    pub fn integrate_lookahead(&mut self) {
        self.improved_path.append(&mut self.lookahead);
    }

    pub fn get_path_iterator(&self) -> impl Iterator<Item = &Node> {
        self.improved_path
            .iter()
            .chain(self.lookahead.iter())
            .chain(self.a_star.iter())
    }
}

#[derive(Clone)]
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
    fn movement_cost(position: (f32, f32), initial_time: f32) -> TextBubble {
        TextBubble {
            text: String::from("+1"),
            color: (1.0, 0.0, 0.0),
            size: 30,
            initial_position: (position.0, position.1 + 0.8),
            travel_lenght: 1.5,
            travel_time: 3.0,
            initial_time,
        }
    }

    fn collision_cost(position: (f32, f32), initial_time: f32) -> TextBubble {
        TextBubble {
            text: format!("+{:?}", COLLISION_COST),
            color: (1.0, 0.0, 0.0),
            size: 40,
            initial_position: (position.0, position.1 + 0.8),
            travel_lenght: 1.5,
            travel_time: 3.0,
            initial_time,
        }
    }
}

impl Warehouse {
    pub fn new(size: (i32, i32), walls: HashSet<Node>) -> Self {
        Warehouse { size, walls }
    }

    pub fn node_is_valid(&self, node: &Node) -> bool {
        node.0 >= 0
            && node.0 < self.size.0
            && node.1 >= 0
            && node.1 < self.size.1
            && !self.walls.contains(node)
    }
}

impl WarehouseSim {
    pub fn new(
        warehouse: Warehouse,
        location: WareSimLocation,
        paths: Vec<AgentPath>,
        speed: f32,
    ) -> Self {
        let n_robots = paths.len();
        let mut warehouse_sim = WarehouseSim {
            warehouse,
            location,
            running: false,
            paths,
            drawn_paths: vec![],
            run_time: 0.0,
            speed,
            bubbles: Vec::new(),
            cumulative_cost: 0,
            collisions: vec![vec![false; n_robots]; n_robots],
        };

        let mut node_availability = std::collections::HashMap::new();
        let drawn_paths = (0..warehouse_sim.paths.len())
            .map(|i| warehouse_sim.generate_path_segments(i, &mut node_availability, 0.0))
            .collect();
        warehouse_sim.drawn_paths = drawn_paths;
        warehouse_sim
    }

    pub fn adjust_for_path(&mut self, coord: (f32, f32), max_size: f32) {
        let agent_path = self.paths.iter().find(|p| !p.lookahead.is_empty()).unwrap();
        let path: Vec<(i32, i32)> = agent_path
            .improved_path
            .iter()
            .chain(agent_path.lookahead.iter())
            .copied()
            .collect();

        let ref_point = (
            agent_path.improved_path.iter().last().unwrap().0 as f32,
            agent_path.improved_path.iter().last().unwrap().1 as f32,
        );

        let min_x = path.iter().map(|x| x.0).min().unwrap() as f32;
        let min_y = path.iter().map(|x| x.1).min().unwrap() as f32;
        let max_x = path.iter().map(|x| x.0).max().unwrap() as f32;
        let max_y = path.iter().map(|x| x.1).max().unwrap() as f32;

        let cell_size_x = max_size / (max_x - min_x).min(self.location.cell_size) as f32;
        let cell_size_y = max_size / (max_y - min_y).min(self.location.cell_size) as f32;

        let cell_size = cell_size_x.min(cell_size_y).min(self.location.cell_size);

        let bounding_center = coord;
        let central_node = (
            self.warehouse.size.0 as f32 / 2.0 - 0.5,
            self.warehouse.size.1 as f32 / 2.0 - 0.5,
        );

        self.location.cell_size = cell_size;
        self.location.position = (
            bounding_center.0 - (ref_point.0 - central_node.0) * cell_size,
            bounding_center.1 - (ref_point.1 - central_node.1) * cell_size,
        );

        // println!(
        //     "Adjustment completed:\n\n\
        //     \tPath:\n{:?}\n\n\
        //     Bounding box:\n{:?}\n\n\
        //     Cell size:\n{:?}\n\n\
        //     Position:\n{:?}\n\n\n\n",
        //     path, bounding_box, self.cell_size, self.position
        // );
    }

    pub fn is_finished(&self) -> bool {
        let turn = (self.run_time * self.speed).floor() as usize;
        self.paths
            .iter()
            .all(|p| p.get_path_iterator().count() <= turn)
    }

    pub fn get_current_cost(&self) -> i32 {
        self.cumulative_cost
    }

    pub fn get_location(&self) -> &WareSimLocation {
        &self.location
    }

    pub fn set_location(&mut self, new_location: WareSimLocation) {
        self.location = new_location;
    }

    pub fn toggle_running(&mut self, running: bool) {
        self.running = running;
    }

    pub fn get_paths(&self) -> &[AgentPath] {
        &self.paths
    }

    pub fn get_mut_paths(&mut self) -> &mut [AgentPath] {
        &mut self.paths
    }

    fn fnode_to_coord(&self, node: &(f32, f32)) -> (f32, f32) {
        let base_node = (
            self.location.position.0
                - self.location.cell_size * ((self.warehouse.size.0 - 1) as f32 / 2.0),
            self.location.position.1
                - self.location.cell_size * ((self.warehouse.size.1 - 1) as f32 / 2.0),
        );

        (
            base_node.0 + node.0 * self.location.cell_size,
            base_node.1 + node.1 * self.location.cell_size,
        )
    }

    fn inode_to_coord(&self, node: &(i32, i32)) -> (f32, f32) {
        self.fnode_to_coord(&(node.0 as f32, node.1 as f32))
    }

    fn get_robot_position(&self, robot_index: usize, time: f32) -> (f32, f32) {
        let path = &self.paths[robot_index];
        let current_turn = time * self.speed;

        // The complete path of the robot is the obatained chaining the 3 parts
        let mut complete_path = path
            .improved_path
            .iter()
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

    pub fn update_time(&mut self, delta_time: f32) {
        if !self.running {
            return;
        }
        let new_time = self.run_time + delta_time;

        let prev_turn = self.run_time * self.speed;
        let new_turn = new_time * self.speed;

        let turn_start = prev_turn == 0.0 || (new_turn.floor() as i32 != prev_turn.floor() as i32);

        for robot_index in 0..self.paths.len() {
            let new_position = self.get_robot_position(robot_index, new_time);

            let new_node = (new_position.0.round() as i32, new_position.1.round() as i32);
            let target_node = self.paths[robot_index].get_path_iterator().last().unwrap();

            if (target_node.0 != new_node.0 || target_node.1 != new_node.1) && turn_start {
                self.cumulative_cost += 1;
                self.bubbles
                    .push(TextBubble::movement_cost(new_position, new_time));
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
                    self.bubbles
                        .push(TextBubble::collision_cost(mean_position, new_time));
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

    pub fn draw_cost(&self, coord: (f32, f32), drawing: &nannou::draw::Draw) {
        drawing
            .text(&format!("{}", self.get_current_cost()))
            .font_size(self.location.cell_size as u32)
            .x_y(coord.0, coord.1);
    }

    pub fn draw(&self, drawing: &nannou::draw::Draw) {
        self.draw_warehouse(drawing);
        self.draw_all_robot_paths(self.run_time, drawing);
        for robot_index in 0..self.paths.len() {
            self.draw_robot(robot_index, self.run_time, drawing);
        }
        // drawing.text("Compramos tu coche").font_size(50).w(100.0);
        for bubble_index in 0..self.bubbles.len() {
            self.draw_bubble(bubble_index, self.run_time, drawing);
        }
    }

    pub fn draw_warehouse(&self, drawing: &nannou::draw::Draw) {
        drawing
            .rect()
            .color(BLACK)
            .x_y(self.location.position.0, self.location.position.1)
            .w(self.location.cell_size * (self.warehouse.size.0 as f32 + 0.2))
            .h(self.location.cell_size * (self.warehouse.size.1 as f32 + 0.2));

        for i in 0..self.warehouse.size.0 {
            for j in 0..self.warehouse.size.1 {
                let (x, y) = self.inode_to_coord(&(i, j));
                let color = if self.warehouse.walls.contains(&(i, j)) {
                    BLACK
                } else if (i + j) % 2 == 0 {
                    get_colour(ColorCollection::GRAY, 0)
                } else {
                    get_colour(ColorCollection::GRAY, 1)
                };
                drawing
                    .rect()
                    .color(color)
                    .x_y(x, y)
                    .w(self.location.cell_size)
                    .h(self.location.cell_size);
            }
        }

        let diameter = self.location.cell_size * 0.7;

        for (agent_index, path) in self.get_paths().iter().enumerate() {
            let last_node = path.get_path_iterator().last().unwrap();
            let target_position = self.inode_to_coord(last_node);
            drawing
                .ellipse()
                .x_y(target_position.0, target_position.1)
                .w_h(diameter, diameter)
                .rgba(0.0, 0.0, 0.0, 0.0)
                .stroke_weight(3.0)
                .stroke(get_colour(ColorCollection::DARK, agent_index));
        }
    }

    pub fn draw_robot_path(
        &self,
        agent_index: usize,
        time: f32,
        end_turn: Option<usize>,
        drawing: &nannou::draw::Draw,
    ) {
        let im_path = &self.drawn_paths[agent_index];

        // The current turn, which is the point in the path in which the robot is,
        // starts at 0 at start_time and increses by "speed" each second.
        let current_turn = (time * self.speed).floor() as usize;
        let alpha = time * self.speed - current_turn as f32;

        let mut last_turn = end_turn.unwrap_or(im_path.len());
        if last_turn == 0 {
            return;
        } else {
            last_turn -= 1;
        };

        for (i, x) in im_path
            .windows(2)
            .take(last_turn)
            .skip(current_turn)
            .enumerate()
        {
            let mut start = self.fnode_to_coord(&x[0].0);
            let original_start = start;
            let end = self.fnode_to_coord(&x[1].0);

            let colour = x[1].1;

            // Chech that they are aligned. If not, create a small 90 deg.
            // corner to fix it, moving the start
            if start.0 != end.0 && start.1 != end.1 {
                let x_dif = (start.0 - end.0).abs();
                let y_dif = (start.1 - end.1).abs();

                if x_dif < y_dif {
                    drawing
                        .line()
                        .start(pt2(start.0, start.1))
                        .end(pt2(end.0, start.1))
                        .weight(self.location.cell_size / 8.0)
                        .color(colour);
                    start.0 = end.0;
                } else {
                    drawing
                        .line()
                        .start(pt2(start.0, start.1))
                        .end(pt2(start.0, end.1))
                        .weight(self.location.cell_size / 8.0)
                        .color(colour);
                    start.1 = end.1;
                }
            }

            if i == 0 {
                start = (
                    (start.0 * (1.0 - alpha) + end.0 * alpha),
                    (start.1 * (1.0 - alpha) + end.1 * alpha),
                );
            }

            if original_start == end {
                drawing
                    .text("Zzz")
                    .x_y(start.0, start.1 + self.location.cell_size * 0.2)
                    .color(colour);
            } else {
                drawing
                    .line()
                    .start(pt2(start.0, start.1))
                    .end(pt2(end.0, end.1))
                    .weight(self.location.cell_size / 8.0)
                    .color(colour);
                // drawing
                //     .tri()
                //     .rotate((end.1 - start.1).atan2(end.0 - start.0))
                //     .x_y((start.0 + end.0) / 2.0, (start.1 + end.1) / 2.0)
                //     .w_h(self.location.cell_size / 2.0, self.location.cell_size / 2.0)
                //     .color(colour);
            }
        }
    }

    fn draw_all_robot_paths(&self, time: f32, drawing: &nannou::draw::Draw) {
        (0..self.paths.len()).for_each(|i| self.draw_robot_path(i, time, None, drawing));
    }

    pub fn draw_robot(&self, robot_index: usize, time: f32, drawing: &nannou::draw::Draw) {
        let diameter = 0.7 * self.location.cell_size;

        let current_node = self.get_robot_position(robot_index, time);

        let position = self.fnode_to_coord(&current_node);
        let colour = if self.collisions[robot_index].iter().any(|&x| x) {
            ORANGE
        } else {
            get_colour(ColorCollection::DARK, robot_index)
        };

        drawing
            .ellipse()
            .color(colour)
            .x_y(position.0, position.1)
            .w_h(diameter, diameter);
    }

    fn draw_bubble(&self, bubble_index: usize, time: f32, drawing: &nannou::draw::Draw) {
        let bubble = &self.bubbles[bubble_index];
        let status: f32 = (time - bubble.initial_time) / bubble.travel_time;
        let position = self.fnode_to_coord(&(
            bubble.initial_position.0,
            bubble.initial_position.1 + bubble.travel_lenght * status,
        ));

        drawing
            .text(&bubble.text)
            .rgba(
                bubble.color.0,
                bubble.color.1,
                bubble.color.2,
                1.0 - status.min(1.0),
            )
            .font_size(bubble.size)
            .x_y(position.0, position.1);
    }

    fn generate_path_segments(
        &self,
        agent_index: usize,
        node_availability: &mut std::collections::HashMap<Node, NodeOverlap>,
        time: f32,
    ) -> Vec<((f32, f32), StdColour)> {
        let path = &self.paths[agent_index];
        let current_turn = time * self.speed;

        let mut path_iterator = path
            .improved_path
            .iter()
            .map(|&n| (n, get_colour(ColorCollection::DARK, agent_index)))
            .chain(path.lookahead.iter().map(|&n| (n, LOOKAHEAD_COLOUR)))
            .chain(
                path.a_star
                    .iter()
                    .map(|&n| (n, get_colour(ColorCollection::LIGHT, agent_index))),
            )
            .skip(current_turn.floor() as usize);

        let Some(buffer_first_virtual_element ) = path_iterator.next() else { return vec![] };
        let Some(second_node) = path_iterator.next() else { return vec![] };

        let mut buffer_first_real_element = (self.get_robot_position(agent_index, time), ORANGE);

        let mut buffer: Vec<((i32, i32), StdColour)> =
            vec![buffer_first_virtual_element, second_node];

        let mut buffer_direction: (i32, i32) = (
            buffer[1].0 .0 - buffer[0].0 .0,
            buffer[1].0 .1 - buffer[0].0 .1,
        );

        let mut output: Vec<((f32, f32), StdColour)> = Vec::new();

        for (new_node, new_colour) in path_iterator {
            let last_node = buffer.iter().last().unwrap().0;
            let segment_direction = (new_node.0 - last_node.0, new_node.1 - last_node.1);

            if segment_direction == buffer_direction {
                buffer.push((new_node, new_colour));
                continue;
            }

            append_segment_to_path(
                &mut output,
                &mut buffer,
                &mut buffer_first_real_element,
                Some((new_node, new_colour)),
                node_availability,
            );

            buffer_direction = (
                buffer[1].0 .0 - buffer[0].0 .0,
                buffer[1].0 .1 - buffer[0].0 .1,
            );
        }

        append_segment_to_path(
            &mut output,
            &mut buffer,
            &mut buffer_first_real_element,
            None,
            node_availability,
        );
        output
    }
}

const PATH_DELTA: [f32; 5] = [0.0, 0.2, -0.2, 0.4, -0.4];

fn append_segment_to_path(
    complete_path: &mut Vec<((f32, f32), StdColour)>,
    buffer: &mut Vec<((i32, i32), StdColour)>,
    buffer_first_real_element: &mut ((f32, f32), StdColour),
    new_element: Option<((i32, i32), StdColour)>,
    node_availability: &mut std::collections::HashMap<Node, NodeOverlap>,
) {
    // Save the last element of the buffer, to be used as the first in the
    // next buffer
    let new_buffer_first_virtual_element = *buffer.iter().last().unwrap();

    // println!("Appeding this segment:\n{:?}", buffer);

    // Create a float version of the buffer, substituting the first element
    // by the real one, which might be displaced
    let mut float_buffer: Vec<((f32, f32), StdColour)> = vec![*buffer_first_real_element];
    float_buffer.extend(
        buffer
            .iter()
            .skip(1)
            .map(|((x, y), c)| ((*x as f32, *y as f32), *c)),
    );

    // Here, "float_buffer " is shifted
    let shape = Shape::new(buffer);
    let mut available_positions = [true; 5];
    for (node, _) in buffer.iter() {
        // TODO: If it is a point, it should avoid both vertical and horizontal
        if shape == Shape::Point {
            continue;
        }
        let Some(avail) = node_availability.get(node) else { continue };

        let avail = match shape {
            Shape::Point => continue,
            Shape::Horizontal => avail.horizontal,
            Shape::Vertical => avail.vertical,
        };

        (0..5).for_each(|i| available_positions[i] = available_positions[i] && avail[i]);
    }

    let delta_mag_index = (0..5).find(|&i| available_positions[i]).expect("RIIIP");

    let delta_mag = PATH_DELTA[delta_mag_index];
    let delta = match shape {
        Shape::Point => (0.0, 0.0),
        Shape::Horizontal => {
            buffer.iter().for_each(|n| {
                let mut current_availability =
                    node_availability.get(&n.0).cloned().unwrap_or_default();
                current_availability.horizontal[delta_mag_index] = false;
                node_availability.insert(n.0, current_availability);
            });
            (0.0, delta_mag)
        }
        Shape::Vertical => {
            buffer.iter().for_each(|n| {
                let mut current_availability =
                    node_availability.get(&n.0).cloned().unwrap_or_default();
                current_availability.vertical[delta_mag_index] = false;
                node_availability.insert(n.0, current_availability);
            });
            (delta_mag, 0.0)
        }
    };

    float_buffer.iter_mut().for_each(|n| {
        n.0 .0 += delta.0;
        n.0 .1 += delta.1;
    });

    // The last element of the buffer is saved for the next buffer
    *buffer_first_real_element = float_buffer.pop().unwrap();

    // Append the float_buffer to the complete path
    complete_path.append(&mut float_buffer);

    match new_element {
        Some(elem) => {
            *buffer = vec![new_buffer_first_virtual_element, elem];
        }
        None => {
            complete_path.push(*buffer_first_real_element);
        }
    }
}

#[derive(PartialEq, Eq)]
enum Shape {
    Vertical,
    Horizontal,
    Point,
}

impl Shape {
    fn new(buffer: &[((i32, i32), StdColour)]) -> Self {
        let direction = (
            buffer[1].0 .0 - buffer[0].0 .0,
            buffer[1].0 .1 - buffer[0].0 .1,
        );

        if direction.0 == 0 && direction.1 == 0 {
            return Shape::Point;
        }
        if direction.0 != 0 && direction.1 == 0 {
            return Shape::Horizontal;
        }
        if direction.0 == 0 && direction.1 != 0 {
            return Shape::Vertical;
        }
        panic!()
    }
}

#[derive(Clone)]
struct NodeOverlap {
    vertical: [bool; 5],
    horizontal: [bool; 5],
}

impl std::default::Default for NodeOverlap {
    fn default() -> Self {
        Self {
            vertical: [true; 5],
            horizontal: [true; 5],
        }
    }
}
