#![allow(unused)]
use crate::datatype::{constraint::Constraint, constraint::Location, edge::Edge, vertex::Vertex};
use crate::multi_agent::{collision::Collision, node::Node};
use crate::single_agent::{astar::astar, dijkstra::compute_heuristics};
use std::cmp::max;
use std::collections::{BinaryHeap, HashMap, HashSet};

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

pub fn cbs(map: &Vec<Vec<u8>>, starts: Vec<Vertex>, goals: Vec<Vertex>) -> Option<Vec<Vertex>> {
    let num_agents = starts.len();
    let mut h_values: Vec<HashMap<Vertex, u16>> = Vec::with_capacity(num_agents);
    for i in 0..num_agents {
        h_values.push(compute_heuristics(&map, goals[i]));
    }

    let mut paths: Vec<Vec<Vertex>> = Vec::with_capacity(num_agents);
    let mut constraints: Vec<Constraint> = Vec::new();
    // TODO: add ability to pass in constraints by adding it to the param
    for i in 0..num_agents {
        let path = match astar(&map, &starts[i], &goals[i], &h_values[i], &constraints) {
            Some(path) => path,
            None => return None, // No solution
        };
        paths.push(path);
    }
    let g_val = get_sum_cost(&paths);
    let collisions = detect_collisions(&paths);
    let root = Node::new(g_val, 0, paths, constraints, collisions);
    println!("{:?}", root);
    None // No solution
}
