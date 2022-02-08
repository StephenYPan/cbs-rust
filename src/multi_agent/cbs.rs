use crate::datatype::{cardinal, collision, constraint, edge, location, vertex};
use crate::map_reader::map;
use crate::multi_agent::{heuristic, lib, mdd};
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

/// Mutates the input vector of collisions by replacing the collisions
/// with cardinal or semi-cardinal collisions if applicable.
fn detect_cardinal_conflicts(collisions: &mut [collision::Collision], mdds: &[mdd::Mdd]) {
    let mut conflict_index: Vec<usize> = Vec::new();
    let mut conflicts: Vec<collision::Collision> = Vec::with_capacity(collisions.len());
    for (i, collision) in collisions.iter_mut().enumerate() {
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;
        if let Some(mut cardinal_conflict) =
            mdd::find_cardinal_conflict(&mdds[a1], &mdds[a2], a1 as u8, a2 as u8)
        {
            // NOTE: The return value may be from cache. To account for the possibility
            // the recursive call of CBS may have generated the cached value we need to
            // adjust the return value's agent 1 and 2 to match the present conflicting
            // agent 1 and 2.
            cardinal_conflict.a1 = collision.a1;
            cardinal_conflict.a2 = collision.a2;
            conflict_index.push(i);
            conflicts.push(cardinal_conflict);
            continue;
        }
        if mdd::find_dependency_conflict(&mdds[a1], &mdds[a2], a1 as u8, a2 as u8) {
            collision.cardinal = cardinal::Cardinal::Semi;
            continue;
        }
    }
    for (i, c) in conflict_index.iter().zip(conflicts) {
        collisions[*i] = c;
    }
}

fn detect_collisions(paths: &[Vec<vertex::Vertex>]) -> Vec<collision::Collision> {
    let mut collisions = Vec::new();
    let num_agents = paths.len();
    for i in 0..num_agents {
        for j in (i + 1)..num_agents {
            for t in 1..max(paths[i].len(), paths[j].len()) {
                let cur_loc1 = get_location(&paths[i], t);
                let cur_loc2 = get_location(&paths[j], t);
                let prev_loc1 = get_location(&paths[i], t - 1);
                let prev_loc2 = get_location(&paths[j], t - 1);
                if cur_loc1 == cur_loc2 {
                    collisions.push(collision::Collision::new(
                        i as u8,
                        j as u8,
                        location::Location::new(cur_loc1),
                        t as u16,
                        cardinal::Cardinal::default(),
                    ));
                    break;
                }
                if cur_loc1 == prev_loc2 && cur_loc2 == prev_loc1 {
                    collisions.push(collision::Collision::new(
                        i as u8,
                        j as u8,
                        location::Location::new(edge::Edge(cur_loc2, cur_loc1)),
                        t as u16,
                        cardinal::Cardinal::default(),
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
    let mut vec = vec![
        constraint::Constraint::new(
            collision.a1,
            collision.loc,
            collision.timestep,
            false,
            collision.cardinal,
        ),
        constraint::Constraint::new(
            collision.a2,
            collision.loc,
            collision.timestep,
            false,
            collision.cardinal,
        ),
    ];
    match collision.loc {
        location::Location::Vertex(_) => vec,
        location::Location::Edge(edge) => {
            vec[1] = constraint::Constraint::new(
                collision.a2,
                location::Location::new(edge::Edge(edge.1, edge.0)),
                collision.timestep,
                false,
                collision.cardinal,
            );
            vec
        }
    }
}

fn disjoint_split(collision: &collision::Collision) -> Vec<constraint::Constraint> {
    let mut result = standard_split(collision);
    // TODO: Convert this to a deterministic picking method
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
            // Vertex violation
            if constraint.loc.1 == cur_loc {
                agents.push(agent);
            }
        } else {
            // Edge violation
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

/// Attempt to bypass non-cardinal conflicts. If bypass was successful
/// path will be edited and new collisions will be computed, otherwise
/// return the first collision in collisions.
fn bypass_collisions(
    node: &mut Node,
    map: &[Vec<u8>],
    starts: &[vertex::Vertex],
    goals: &[vertex::Vertex],
    h_values: &[HashMap<vertex::Vertex, u16>],
) -> Option<collision::Collision> {
    let mut num_collisions = node.collisions.len();
    for collision in &node.collisions {
        let constraints = standard_split(collision);
        for constraint in constraints {
            let agent = constraint.agent as usize;
            let mut temp_constraints: Vec<constraint::Constraint> = vec![constraint];
            temp_constraints.extend(&node.constraints);
            let agent_constraints: Vec<constraint::Constraint> = temp_constraints
                .iter()
                .filter(|c| c.agent == constraint.agent)
                .copied()
                .collect();
            match astar::astar(
                map,
                &starts[agent],
                &goals[agent],
                &h_values[agent],
                &agent_constraints,
                node.paths[agent].len() - 1,
            ) {
                Some(path) => {
                    // Edit path iff new path is same length as old path
                    // and the number of collisions has decreased.
                    if node.paths[agent].len() == path.len() {
                        let mut new_paths = node.paths.clone();
                        new_paths[agent] = path.clone();
                        let new_num_collisions = detect_collisions(&new_paths).len();
                        if new_num_collisions < num_collisions {
                            node.paths[agent] = path;
                            num_collisions = new_num_collisions;
                        }
                    }
                }
                None => continue, // Constraint yields no solution, skip
            };
        }
    }
    node.collisions = detect_collisions(&node.paths);
    node.g_val = get_sum_cost(&node.paths);
    if node.collisions.is_empty() {
        None // Found solution
    } else {
        Some(node.collisions[0])
    }
}

pub fn cbs(
    map_instance: &map::MapInstance,
    constraints: Option<Vec<constraint::Constraint>>,
    child_process: bool,
    disjoint: bool,
    heuristics: Vec<bool>,
) -> Option<Vec<Vec<vertex::Vertex>>> {
    let now = Instant::now();
    let mut mdd_time = std::time::Duration::new(0, 0);
    let mut heuristic_time = std::time::Duration::new(0, 0);

    let map = &map_instance.map;
    let starts = &map_instance.starts;
    let goals = &map_instance.goals;

    let num_agents = starts.len();
    let mut h_values: Vec<HashMap<vertex::Vertex, u16>> = Vec::with_capacity(num_agents);
    for g in goals {
        h_values.push(dijkstra::compute_heuristics(map, *g));
    }

    let mut root_paths: Vec<Vec<vertex::Vertex>> = Vec::with_capacity(num_agents);
    let mut root_mdds: Vec<mdd::Mdd> = Vec::with_capacity(num_agents);
    for i in 0..num_agents {
        let agent_constraints: Vec<constraint::Constraint> = match &constraints {
            Some(some_constraints) => some_constraints
                .iter() // parallelize iter(?)
                .filter(|c| c.agent == i as u8)
                .copied()
                .collect(),
            None => Vec::new(),
        };
        match astar::astar(
            map,
            &starts[i],
            &goals[i],
            &h_values[i],
            &agent_constraints,
            0,
        ) {
            Some(path) => root_paths.push(path),
            None => return None, // No solution
        };
        let mdd_now = Instant::now();
        root_mdds.push(mdd::Mdd::new(map, &root_paths[i], &agent_constraints));
        mdd_time += mdd_now.elapsed();
    }

    let root_constraints: Vec<constraint::Constraint> = match constraints {
        Some(constraints) => constraints,
        None => Vec::new(),
    };
    let mut root_collisions = detect_collisions(&root_paths);
    detect_cardinal_conflicts(&mut root_collisions, &root_mdds);
    let root_g_val = get_sum_cost(&root_paths);
    let heuristic_now = Instant::now();
    let root_h_val = match heuristics.as_slice() {
        [true, false, false] => heuristic::cg_heuristic(&root_collisions, &root_mdds),
        [_, true, false] => heuristic::dg_heuristic(&root_collisions, &root_mdds),
        [_, _, true] => heuristic::wdg_heuristic(
            map_instance,
            &root_constraints,
            &root_collisions,
            &root_mdds,
        ),
        _ => 0,
    };
    heuristic_time += heuristic_now.elapsed();

    // Move all the variables inside the node.
    let root = Node::new(
        root_g_val,
        root_h_val,
        root_paths,
        root_constraints,
        root_collisions,
        root_mdds,
    );

    let mut pop_counter: usize = 0;
    let mut push_counter: usize = 0;

    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    open_list.push(root);
    push_counter += 1;

    while !open_list.is_empty() {
        let mut cur_node = open_list.pop().unwrap();
        pop_counter += 1;
        if !child_process {
            println!(
                "pop: [f-val: {}, g-val: {}, h-val: {}, num_col: {:2}]",
                cur_node.g_val + cur_node.h_val,
                cur_node.g_val,
                cur_node.h_val,
                cur_node.collisions.len()
            );
        }
        if cur_node.collisions.is_empty() {
            // Solution found
            let elapsed_time = now.elapsed();
            if !child_process {
                println!("\nCPU time: {:>17?}", elapsed_time);
                println!("Mdd build time: {:>11?}", mdd_time);
                println!("Heuristic time: {:>11?}", heuristic_time);
                println!("Cost: {}", cur_node.g_val);
                println!("Nodes expanded:  {}", pop_counter);
                println!("Nodes generated: {}", push_counter);
            }
            return Some(cur_node.paths);
        }
        // TODO: Lazy heuristics

        // TODO: Meta-agent

        // Improved cbs: Always split on cardinal or semi-cardinal conflicts, then
        // attempt to bypass when there are no more cardinal and semi-cardinal conflicts.
        let mut cur_collision: collision::Collision = cur_node.collisions[0];
        let mut attempt_bypass = true;
        for collision in &cur_node.collisions {
            match collision.cardinal {
                cardinal::Cardinal::Full | cardinal::Cardinal::Semi => {
                    cur_collision = *collision;
                    attempt_bypass = false;
                    break;
                }
                cardinal::Cardinal::Non => {}
            }
        }
        if attempt_bypass {
            match bypass_collisions(&mut cur_node, map, starts, goals, &h_values) {
                Some(collision) => cur_collision = collision,
                None => {
                    open_list.push(cur_node);
                    push_counter += 1;
                    continue;
                }
            };
        }
        let new_constraints = if disjoint {
            disjoint_split(&cur_collision)
        } else {
            standard_split(&cur_collision)
        };
        for new_constraint in new_constraints {
            let constraint_agent = new_constraint.agent as usize;
            let mut new_constraints: Vec<constraint::Constraint> = Vec::new();
            new_constraints.extend(&cur_node.constraints);
            new_constraints.push(new_constraint);

            let mut new_mdds: Vec<mdd::Mdd> = Vec::with_capacity(num_agents);
            new_mdds.clone_from(&cur_node.mdds); // Clone parent, edits will not overwrite parent.

            let agent_constraints: Vec<constraint::Constraint> = new_constraints
                .iter() // parallelize iter(?)
                .filter(|c| c.agent == new_constraint.agent)
                .copied()
                .collect();
            let mut new_paths = cur_node.paths.clone();
            let min_path_length = match new_constraint.cardinal {
                cardinal::Cardinal::Full => {
                    if new_constraint.is_positive {
                        new_paths[constraint_agent].len() - 1
                    } else if new_constraint.is_edge {
                        // Edge collisions requires 2 additional moves to resolve
                        new_paths[constraint_agent].len() + 1
                    } else {
                        new_paths[constraint_agent].len()
                    }
                }
                cardinal::Cardinal::Semi => {
                    // NOTE: Why can't the condition be constraint.is_positive?
                    if disjoint {
                        new_paths[constraint_agent].len() - 1
                    } else {
                        new_paths[constraint_agent].len()
                    }
                }
                cardinal::Cardinal::Non => 0,
            };
            match astar::astar(
                map,
                &starts[constraint_agent],
                &goals[constraint_agent],
                &h_values[constraint_agent],
                &agent_constraints,
                min_path_length,
            ) {
                Some(p) => new_paths[constraint_agent] = p,
                None => continue, // New constraint yields no solution
            };
            // Update mdd given the new constraints
            let mdd_now = Instant::now();
            new_mdds[constraint_agent] =
                mdd::Mdd::new(map, &new_paths[constraint_agent], &agent_constraints);
            mdd_time += mdd_now.elapsed();

            // Check for positive constraint
            if new_constraint.is_positive {
                let violating_agents = paths_violating_pos_constraint(&new_constraint, &new_paths);
                let loc: location::Location = if !new_constraint.is_edge {
                    location::Location::new(new_constraint.loc.1)
                } else {
                    location::Location::new(edge::Edge(new_constraint.loc.1, new_constraint.loc.0))
                };
                let mut invalid_pos_constraint = false;
                for violating_agent in violating_agents {
                    new_constraints.push(constraint::Constraint::new(
                        violating_agent as u8,
                        loc,
                        new_constraint.timestep,
                        false,
                        cardinal::Cardinal::default(),
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
                        0,
                    ) {
                        Some(p) => {
                            new_paths[violating_agent] = p;
                        }
                        None => {
                            invalid_pos_constraint = true;
                            break;
                        }
                    };
                    // update agent's mdd
                    let mdd_now = Instant::now();
                    new_mdds[violating_agent] = mdd::Mdd::new(
                        map,
                        &new_paths[violating_agent],
                        &violating_agent_constraints,
                    );
                    mdd_time += mdd_now.elapsed();
                }
                if invalid_pos_constraint {
                    // Positive constraint yields no solution
                    continue;
                }
            }
            new_constraints.shrink_to_fit();
            let mut new_collisions = detect_collisions(&new_paths);
            detect_cardinal_conflicts(&mut new_collisions, &new_mdds);
            let new_g_val = get_sum_cost(&new_paths);
            let heuristic_now = Instant::now();
            let new_h_val = match heuristics.as_slice() {
                [true, false, false] => heuristic::cg_heuristic(&new_collisions, &new_mdds),
                [_, true, false] => heuristic::dg_heuristic(&new_collisions, &new_mdds),
                [_, _, true] => heuristic::wdg_heuristic(
                    map_instance,
                    &new_constraints,
                    &new_collisions,
                    &new_mdds,
                ),
                _ => 0,
            };
            heuristic_time += heuristic_now.elapsed();

            // Move all the variables inside the node.
            let new_node = Node::new(
                new_g_val,
                new_h_val,
                new_paths,
                new_constraints,
                new_collisions,
                new_mdds,
            );
            open_list.push(new_node);
            push_counter += 1;
        }
    }
    None // No solution
}

use std::cmp::Ordering;

#[derive(Debug, Eq)]
struct Node {
    pub g_val: u16,
    pub h_val: u16,
    pub paths: Vec<Vec<vertex::Vertex>>,
    pub constraints: Vec<constraint::Constraint>,
    pub collisions: Vec<collision::Collision>,
    pub mdds: Vec<mdd::Mdd>,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        if self.g_val.eq(&other.g_val) {
            lib::hash2d(&self.paths) == lib::hash2d(&other.paths)
        } else {
            false
        }
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        let f_val = self.g_val + self.h_val;
        let other_f_val = other.g_val + other.h_val;
        if f_val == other_f_val {
            // Tie-breaking method
            self.collisions.len().cmp(&other.collisions.len()).reverse()
        } else {
            f_val.cmp(&other_f_val).reverse()
        }
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
        mdds: Vec<mdd::Mdd>,
    ) -> Node {
        Node {
            g_val,
            h_val,
            paths,
            constraints,
            collisions,
            mdds,
        }
    }
}
