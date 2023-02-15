use super::warehouse::{AgentPath, Node, Warehouse};

use std::collections::HashMap;

pub fn initialize_paths(warehouse: &Warehouse, endpoints: &[(Node, Node)]) -> Vec<AgentPath> {
    endpoints
        .iter()
        .map(|(origin, target)| {
            let mut path = obtain_a_star_path(warehouse, origin, target).unwrap();
            let a_star_path = path.split_off(1);
            AgentPath::new(vec![path[0]], vec![], a_star_path)
        })
        .collect()
}

pub fn improve_policy(
    warehouse: &Warehouse,
    current_policy: &[AgentPath],
    agent: usize,
) -> Vec<Vec<AgentPath>> {
    [(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]
        .iter()
        .filter_map(|delta| {
            let mut new_policy: Vec<AgentPath> = current_policy.to_owned();
            let initial_node = current_policy[agent].improved_path.iter().last().unwrap();
            let target_node = new_policy[agent].get_path_iterator().last().unwrap();
            let lookahead_node = (initial_node.0 + delta.0, initial_node.1 + delta.1);

            if !warehouse.node_is_valid(&lookahead_node) {
                return None;
            }

            let a_star_path = obtain_a_star_path(warehouse, &lookahead_node, target_node)
                .unwrap()
                .split_off(1);

            new_policy[agent].lookahead = vec![lookahead_node];
            new_policy[agent].a_star = a_star_path;

            Some(new_policy)
        })
        .collect()
}

struct QueueNode {
    node: Node,
    cost_to_target: i32,
    heuristic_total_cost: i32,
    parent: Node,
}

fn obtain_a_star_path(warehouse: &Warehouse, origin: &Node, target: &Node) -> Option<Vec<Node>> {
    let mut queue = vec![QueueNode {
        node: *target,
        cost_to_target: 0,
        heuristic_total_cost: 0,
        parent: *target,
    }];
    let mut expanded_nodes: HashMap<Node, Node> = HashMap::new();

    loop {
        let parent_queue_node = {
            let index = queue
                .iter()
                .enumerate()
                .min_by_key(|(_, x)| x.heuristic_total_cost)?
                .0;
            queue.swap_remove(index)
        };
        let parent_node = parent_queue_node.node;

        if expanded_nodes.contains_key(&parent_node) {
            continue;
        }

        let children = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            .iter()
            .map(|(delta_x, delta_y)| (parent_node.0 + delta_x, parent_node.1 + delta_y));

        for child in children {
            if !warehouse.node_is_valid(&child) {
                continue;
            }

            let heuristic_distance_left = (child.0 - origin.0).abs() + (child.1 - origin.1).abs();
            let cost_to_target = parent_queue_node.cost_to_target + 1;
            let heuristic_total_cost = cost_to_target + heuristic_distance_left;

            queue.push(QueueNode {
                node: child,
                cost_to_target,
                heuristic_total_cost,
                parent: parent_node,
            });
        }

        expanded_nodes.insert(parent_node, parent_queue_node.parent);

        if parent_node == *origin {
            break;
        }
    }

    // Obtain the path
    let mut path = vec![*origin];
    loop {
        let last_node = path.iter().last().unwrap();
        if last_node == target {
            break;
        }
        let next_node = expanded_nodes.get(last_node).unwrap();
        path.push(*next_node);
    }

    Some(path)
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::walls;

    #[test]
    fn basic_test() {
        let warehouse = Warehouse::new((6, 8), walls! {});
        let origin = (2, 3);

        for x in 0..6 {
            for y in 0..8 {
                let path = obtain_a_star_path(&warehouse, &origin, &(x, y));
                let min_dist = (origin.0 - x).abs() + (origin.1 - y).abs();

                assert_eq!(path.unwrap().len() - 1, min_dist as usize);
            }
        }
    }

    #[test]
    fn sligthly_tricky() {
        let wh_walls = walls! {
            (1,1), (2,1), (3,1), (4,1), (1,2), (3,3), (1,4)
        };
        let warehouse = Warehouse::new((5, 5), wh_walls);

        let path_1 = obtain_a_star_path(&warehouse, &(4, 4), &(4, 0)).unwrap();
        let path_2 = obtain_a_star_path(&warehouse, &(4, 0), &(4, 4)).unwrap();

        assert_eq!(
            path_1,
            vec![
                (4, 4),
                (3, 4),
                (2, 4),
                (2, 3),
                (1, 3),
                (0, 3),
                (0, 2),
                (0, 1),
                (0, 0),
                (1, 0),
                (2, 0),
                (3, 0),
                (4, 0)
            ]
        );
        assert_eq!(path_1, path_2.into_iter().rev().collect::<Vec<_>>());
    }
}
