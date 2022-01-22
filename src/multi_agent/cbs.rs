use crate::datatype::{collision, constraint, edge, mdd, vertex};
use crate::single_agent::{astar, dijkstra};
use std::cmp::max;
use std::collections::{BinaryHeap, HashMap};
use std::time::Instant;

// use rayon::prelude::*;

fn get_sum_cost(paths: &[Vec<vertex::Vertex>]) -> u16 {
    paths.iter().map(|p| (p.len() - 1) as u16).sum()
}

fn get_location(path: &[vertex::Vertex], timestep: usize) -> vertex::Vertex {
    let path_len = path.len();
    if timestep < path_len {
        path[max(0, timestep)]
    } else {
        path[path_len - 1]
    }
}

fn detect_collisions(paths: &[Vec<vertex::Vertex>]) -> Vec<collision::Collision> {
    let mut collisions = Vec::new();
    let num_agents = paths.len();
    for i in 0..num_agents {
        for j in i + 1..num_agents {
            for t in 1..max(paths[i].len(), paths[j].len()) {
                let cur_loc1 = get_location(&paths[i], t);
                let cur_loc2 = get_location(&paths[j], t);
                let prev_loc1 = get_location(&paths[i], t - 1);
                let prev_loc2 = get_location(&paths[j], t - 1);
                if cur_loc1 == cur_loc2 {
                    // vertex::Vertex collision
                    collisions.push(collision::Collision::new(
                        i as u8,
                        j as u8,
                        constraint::Location::new(cur_loc1),
                        t as u16,
                    ));
                    break;
                }
                if cur_loc1 == prev_loc2 && cur_loc2 == prev_loc1 {
                    // edge::Edge collision
                    collisions.push(collision::Collision::new(
                        i as u8,
                        j as u8,
                        constraint::Location::new(edge::Edge(cur_loc2, cur_loc1)),
                        t as u16,
                    ));
                    break;
                }
            }
        }
    }
    collisions.shrink_to_fit();
    collisions
}

fn standard_split(collision: &collision::Collision) -> Vec<constraint::Constraint> {
    match collision.loc {
        constraint::Location::Vertex(_) => {
            vec![
                constraint::Constraint::new(collision.a1, collision.loc, collision.timestep, false),
                constraint::Constraint::new(collision.a2, collision.loc, collision.timestep, false),
            ]
        }
        constraint::Location::Edge(edge) => {
            let reversed_edge = constraint::Location::new(edge::Edge(edge.1, edge.0));
            vec![
                constraint::Constraint::new(collision.a1, collision.loc, collision.timestep, false),
                constraint::Constraint::new(collision.a2, reversed_edge, collision.timestep, false),
            ]
        }
    }
}

fn disjoint_split(collision: &collision::Collision) -> Vec<constraint::Constraint> {
    let mut result = standard_split(collision);
    // Pick a random agent.
    let random_idx = (Instant::now().elapsed().as_nanos() % 2) as usize;
    let random_agent = result[random_idx].agent;
    let random_loc = result[random_idx].loc;
    // Modify the other agent.
    let other_idx = (random_idx + 1) % 2;
    result[other_idx].agent = random_agent;
    result[other_idx].loc = random_loc;
    result[other_idx].is_positive = true;
    result
}

fn paths_violating_pos_constraint(
    constraint: &constraint::Constraint,
    paths: &[Vec<vertex::Vertex>],
) -> Vec<usize> {
    assert!(constraint.is_positive);
    let mut agents = Vec::with_capacity(paths.len());
    for (agent, path) in paths.iter().enumerate() {
        if agent as u8 == constraint.agent {
            continue;
        }
        let cur_loc = get_location(path, constraint.timestep as usize);
        let prev_loc = get_location(path, (constraint.timestep - 1) as usize);
        if !constraint.is_edge {
            // vertex::Vertex violation
            if constraint.loc.1 == cur_loc {
                agents.push(agent);
            }
        } else {
            // edge::Edge violation
            if constraint.loc.0 == prev_loc
                || constraint.loc.1 == cur_loc
                || constraint.loc == edge::Edge(cur_loc, prev_loc)
            {
                agents.push(agent);
            }
        }
    }
    agents
}

pub fn cbs(
    map: &[Vec<u8>],
    starts: Vec<vertex::Vertex>,
    goals: Vec<vertex::Vertex>,
    constraints: Option<Vec<constraint::Constraint>>,
    disjoint: bool,
) -> Option<Vec<Vec<vertex::Vertex>>> {
    let now = Instant::now();

    let num_agents = starts.len();
    let mut h_values: Vec<HashMap<vertex::Vertex, u16>> = Vec::with_capacity(num_agents);
    for g in &goals {
        h_values.push(dijkstra::compute_heuristics(map, *g));
    }

    let mut root_paths: Vec<Vec<vertex::Vertex>> = Vec::with_capacity(num_agents);
    for i in 0..num_agents {
        let agent_constraints: Vec<constraint::Constraint> = match &constraints {
            Some(some_constraints) => some_constraints
                .iter() // parallelize iter
                .filter(|c| c.agent == i as u8)
                .copied()
                .collect(),
            None => Vec::new(),
        };
        match astar::astar(map, &starts[i], &goals[i], &h_values[i], &agent_constraints) {
            Some(path) => root_paths.push(path),
            None => return None, // No solution
        };
    }
    let root_constraints: Vec<constraint::Constraint> = match constraints {
        Some(constraints) => constraints,
        None => Vec::new(),
    };
    let root_collisions = detect_collisions(&root_paths);
    let root_g_val = get_sum_cost(&root_paths);
    let root_h_val = 0;

    // Move all the variables inside the node.
    let root = Node::new(
        root_g_val,
        root_h_val,
        root_paths,
        root_constraints,
        root_collisions,
    );

    let mut pop_counter = 0;
    let mut push_counter = 0;

    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    open_list.push(root);
    push_counter += 1;

    while !open_list.is_empty() {
        let cur_node = open_list.pop().unwrap();
        pop_counter += 1;
        if cur_node.collisions.is_empty() {
            // Solution found
            let elapsed_time = now.elapsed();
            println!("CPU time: {:?}", elapsed_time);
            println!("Cost: {}", cur_node.g_val);
            println!("Nodes expanded:  {}", pop_counter);
            println!("Nodes generated: {}", push_counter);
            return Some(cur_node.paths);
        }
        // TODO: Figure out cardinal collision
        // TODO: Try collision bypass if non-cardinal
        let cur_collision = &cur_node.collisions[0];
        let new_constraints = if disjoint {
            disjoint_split(cur_collision)
        } else {
            standard_split(cur_collision)
        };
        // TODO: Add one thread for constraint checking.
        // Store result node in 2 slot vectors
        // and push them to open_list after thread.join
        for new_constraint in new_constraints {
            let constraint_agent = new_constraint.agent as usize;
            let mut new_constraints: Vec<constraint::Constraint> = vec![new_constraint];
            new_constraints.extend(&cur_node.constraints);
            let agent_constraints: Vec<constraint::Constraint> = new_constraints
                .iter() // parallelize iter
                .filter(|c| c.agent == constraint_agent as u8)
                .copied()
                .collect();
            let mut new_paths = cur_node.paths.clone();
            match astar::astar(
                map,
                &starts[constraint_agent],
                &goals[constraint_agent],
                &h_values[constraint_agent],
                &agent_constraints,
            ) {
                Some(p) => new_paths[constraint_agent] = p,
                None => continue, // New constraint yields no solution
            };

            // Check for positive constraint
            if new_constraint.is_positive {
                let violating_agents = paths_violating_pos_constraint(&new_constraint, &new_paths);
                let loc: constraint::Location = if !new_constraint.is_edge {
                    constraint::Location::new(new_constraint.loc.1)
                } else {
                    constraint::Location::new(edge::Edge(
                        new_constraint.loc.1,
                        new_constraint.loc.0,
                    ))
                };
                let mut invalid_pos_constraint = false;
                for violating_agent in violating_agents {
                    new_constraints.push(constraint::Constraint::new(
                        violating_agent as u8,
                        loc,
                        new_constraint.timestep,
                        false,
                    ));
                    let violating_agent_constraints: Vec<constraint::Constraint> = new_constraints
                        .iter() // parallelize iter
                        .filter(|c| c.agent == violating_agent as u8)
                        .copied()
                        .collect();
                    match astar::astar(
                        map,
                        &starts[violating_agent],
                        &goals[violating_agent],
                        &h_values[violating_agent],
                        &violating_agent_constraints,
                    ) {
                        Some(p) => {
                            new_paths[violating_agent] = p;
                        }
                        None => {
                            invalid_pos_constraint = true;
                            break;
                        }
                    };
                }
                if invalid_pos_constraint {
                    // Positive constraint yields no solution
                    continue;
                }
            }

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
            push_counter += 1;
        }
    }
    None // No solution
}

use std::cmp::Ordering;

#[derive(Debug, Eq)]
pub struct Node {
    pub g_val: u16,
    pub h_val: u16,
    pub paths: Vec<Vec<vertex::Vertex>>,
    pub constraints: Vec<constraint::Constraint>,
    pub collisions: Vec<collision::Collision>,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        if self.g_val.eq(&other.g_val) {
            let paths: Vec<vertex::Vertex> =
                self.paths.iter().flat_map(|v| v.iter()).copied().collect();
            let other_paths: Vec<vertex::Vertex> =
                other.paths.iter().flat_map(|v| v.iter()).copied().collect();
            for (v1, v2) in paths.iter().zip(other_paths.iter()) {
                if *v1 != *v2 {
                    return false;
                }
            }
            true
        } else {
            false
        }
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        let f_val = self.g_val + self.h_val;
        let other_f_val = other.g_val + other.h_val;
        f_val.cmp(&other_f_val).reverse()
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Node {
    pub fn new(
        g_val: u16,
        h_val: u16,
        paths: Vec<Vec<vertex::Vertex>>,
        constraints: Vec<constraint::Constraint>,
        collisions: Vec<collision::Collision>,
    ) -> Node {
        Node {
            g_val,
            h_val,
            paths,
            constraints,
            collisions,
        }
    }
}
