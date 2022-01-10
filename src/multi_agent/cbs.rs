use crate::datatype::{constraint::Constraint, constraint::Location, edge::Edge, vertex::Vertex};
use crate::multi_agent::{collision::Collision, node::Node};
use crate::single_agent::{astar::astar, dijkstra::compute_heuristics};
use std::cmp::max;
use std::collections::{BinaryHeap, HashMap};

fn get_sum_cost(paths: &Vec<Vec<Vertex>>) -> u16 {
    paths.iter().map(|p| p.len() as u16).sum()
}

fn get_location(path: &Vec<Vertex>, timestep: usize) -> Vertex {
    let path_len = path.len();
    if timestep < path_len {
        path[max(0, timestep)]
    } else {
        path[path_len - 1]
    }
}

fn detect_collisions(paths: &Vec<Vec<Vertex>>) -> Vec<Collision> {
    let mut collisions = Vec::new();
    let num_agents = paths.len();
    for i in 0..num_agents {
        for j in i + 1..num_agents {
            for t in 1..max(paths[i].len(), paths[j].len()) {
                let cur_loc1 = get_location(&paths[i], t);
                let cur_loc2 = get_location(&paths[j], t);
                let prev_loc1 = get_location(&paths[i], t - 1);
                let prev_loc2 = get_location(&paths[j], t - 1);
                if prev_loc1 == prev_loc2 {
                    // Vertex collision
                    collisions.push(Collision::new(
                        i as u8,
                        j as u8,
                        Location::new(prev_loc1),
                        (t - 1) as u16,
                    ));
                    break;
                }
                if cur_loc1 == prev_loc2 && cur_loc2 == prev_loc1 {
                    // Edge collision
                    collisions.push(Collision::new(
                        i as u8,
                        j as u8,
                        Location::new(Edge(cur_loc2, cur_loc1)),
                        (t - 1) as u16,
                    ));
                    break;
                }
            }
        }
    }
    collisions
}

fn standard_split(c: &Collision) -> Vec<Constraint> {
    match c.loc {
        Location::Vertex(_) => {
            vec![
                Constraint::new(c.a1, c.loc, c.timestep, false),
                Constraint::new(c.a2, c.loc, c.timestep, false),
            ]
        }
        Location::Edge(e) => {
            let rev_e = Location::new(Edge(e.1, e.0));
            vec![
                Constraint::new(c.a1, c.loc, c.timestep, false),
                Constraint::new(c.a2, rev_e, c.timestep, false),
            ]
        }
    }
}

// fn disjoint_split(c: &Collision) -> Vec<Constraint> {}

pub fn cbs(
    map: &Vec<Vec<u8>>,
    starts: Vec<Vertex>,
    goals: Vec<Vertex>,
) -> Option<Vec<Vec<Vertex>>> {
    let num_agents = starts.len();
    let mut h_values: Vec<HashMap<Vertex, u16>> = Vec::with_capacity(num_agents);
    for i in 0..num_agents {
        h_values.push(compute_heuristics(&map, goals[i]));
    }

    let mut root_paths: Vec<Vec<Vertex>> = Vec::with_capacity(num_agents);
    #[allow(unused_mut)]
    let mut root_constraints: Vec<Constraint> = Vec::new();
    // TODO: add ability to pass in constraints by adding it to the param
    for i in 0..num_agents {
        let agent_constraints: Vec<Constraint> = root_constraints
            .iter()
            .filter(|c| c.agent == i as u8)
            .copied()
            .collect();
        let path = match astar(
            &map,
            &starts[i],
            &goals[i],
            &h_values[i],
            &agent_constraints,
        ) {
            Some(path) => path,
            None => return None, // No solution
        };
        root_paths.push(path);
    }
    let root_g_val = get_sum_cost(&root_paths);
    let root_h_val = 0;
    let root_collisions = detect_collisions(&root_paths);
    let root = Node::new(
        root_g_val,
        root_h_val,
        root_paths,
        root_constraints,
        root_collisions,
    );

    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    open_list.push(root);

    while !open_list.is_empty() {
        let cur_node = open_list.pop().unwrap();
        if cur_node.collisions.len() == 0 {
            // Solution found
            return Some(cur_node.paths);
        }
        // TODO: Figure out cardinal collision
        // TODO: Try collision bypass if non-cardinal
        let collision = &cur_node.collisions[0];
        let constraints = standard_split(collision);
        // TODO: Add one thread for constraint checking.
        // Store result node in 2 slot vectors
        // and push them to open_list after thread.join
        for constraint in constraints {
            let agent = constraint.agent as usize;
            let mut new_constraints: Vec<Constraint> = vec![constraint];
            new_constraints.extend(cur_node.constraints.clone());
            let agent_constraints: Vec<Constraint> = new_constraints
                .iter()
                .filter(|c| c.agent == agent as u8)
                .copied()
                .collect();
            let new_path = match astar(
                &map,
                &starts[agent],
                &goals[agent],
                &h_values[agent],
                &agent_constraints,
            ) {
                Some(p) => p,
                None => continue, // New constraint yields no solution
            };

            let mut new_paths = cur_node.paths.clone();
            new_paths[agent] = new_path;

            let new_collisions = detect_collisions(&new_paths);
            let new_g_val = get_sum_cost(&new_paths);
            let new_h_val = 0;

            // Move all the variables inside the node.
            let new_node = Node::new(
                new_g_val,
                new_h_val,
                new_paths,
                new_constraints,
                new_collisions,
            );

            open_list.push(new_node);
        }
    }

    None // No solution
}
