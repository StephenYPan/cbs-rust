use crate::datatype::{cardinal, collision, constraint, edge, mdd, vertex};
use crate::single_agent::{astar, dijkstra};
use std::cmp::max;
use std::collections::{hash_map::DefaultHasher, BinaryHeap, HashMap, HashSet};
use std::hash::{Hash, Hasher};
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
        for j in (i + 1)..num_agents {
            for t in 1..max(paths[i].len(), paths[j].len()) {
                let cur_loc1 = get_location(&paths[i], t);
                let cur_loc2 = get_location(&paths[j], t);
                let prev_loc1 = get_location(&paths[i], t - 1);
                let prev_loc2 = get_location(&paths[j], t - 1);
                if cur_loc1 == cur_loc2 {
                    // Vertex collision
                    collisions.push(collision::Collision::new(
                        i as u8,
                        j as u8,
                        constraint::Location::new(cur_loc1),
                        t as u16,
                        cardinal::Cardinal::default(),
                    ));
                    break;
                }
                if cur_loc1 == prev_loc2 && cur_loc2 == prev_loc1 {
                    // Edge collision
                    collisions.push(collision::Collision::new(
                        i as u8,
                        j as u8,
                        constraint::Location::new(edge::Edge(cur_loc2, cur_loc1)),
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
    match collision.loc {
        constraint::Location::Vertex(_) => {
            vec![
                constraint::Constraint::new(
                    collision.a1,
                    collision.loc,
                    collision.timestep,
                    false,
                    collision.conflict,
                ),
                constraint::Constraint::new(
                    collision.a2,
                    collision.loc,
                    collision.timestep,
                    false,
                    collision.conflict,
                ),
            ]
        }
        constraint::Location::Edge(edge) => {
            let reversed_edge = constraint::Location::new(edge::Edge(edge.1, edge.0));
            vec![
                constraint::Constraint::new(
                    collision.a1,
                    collision.loc,
                    collision.timestep,
                    false,
                    collision.conflict,
                ),
                constraint::Constraint::new(
                    collision.a2,
                    reversed_edge,
                    collision.timestep,
                    false,
                    collision.conflict,
                ),
            ]
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
    result[other_idx].conflict = cardinal::Cardinal::Semi;
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
                .filter(|c| c.agent == agent as u8)
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

/// Mutates the input vector of collisions by replacing the collisions
/// with cardinal or semi-cardinal collisions if applicable.
fn detect_cardinal_conflicts(node: &mut Node) {
    let mut conflict_index: Vec<usize> = Vec::new();
    let mut conflicts: Vec<collision::Collision> = Vec::new();

    let mut mdd_hashes: HashMap<usize, u64> = HashMap::new();
    let agents: HashSet<usize> = node
        .collisions
        .iter()
        .flat_map(|c| vec![c.a1 as usize, c.a2 as usize])
        .collect();
    for i in agents.iter() {
        let mut state = DefaultHasher::new();
        for layer in &node.mdds[*i].mdd {
            layer.hash(&mut state);
        }
        let hash = state.finish();
        mdd_hashes.insert(*i, hash);
    }

    for (i, collision) in node.collisions.iter_mut().enumerate() {
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;
        let joint_mdd_hash = mdd_hashes[&a1] ^ mdd_hashes[&a2];

        if let Some(cardinal_conflict) = mdd::find_cardinal_conflict(
            &node.mdds[a1],
            &node.mdds[a2],
            a1 as u8,
            a2 as u8,
            joint_mdd_hash,
        ) {
            conflict_index.push(i);
            conflicts.push(cardinal_conflict);
            continue;
        }
        if let Some(semi_cardinal_conflict) = mdd::find_dependency_conflict(
            &node.mdds[a1],
            &node.mdds[a2],
            a1 as u8,
            a2 as u8,
            joint_mdd_hash,
        ) {
            conflict_index.push(i);
            conflicts.push(semi_cardinal_conflict);
            continue;
        }
    }
    for (i, c) in conflict_index.iter().zip(conflicts) {
        node.collisions[*i] = c;
    }
}

pub fn cbs(
    map: &[Vec<u8>],
    starts: Vec<vertex::Vertex>,
    goals: Vec<vertex::Vertex>,
    constraints: Option<Vec<constraint::Constraint>>,
    disjoint: bool,
) -> Option<Vec<Vec<vertex::Vertex>>> {
    let now = Instant::now();
    let mut mdd_time: std::time::Duration = std::time::Duration::new(0, 0);
    let mut col_time: std::time::Duration = std::time::Duration::new(0, 0);

    let num_agents = starts.len();
    let mut h_values: Vec<HashMap<vertex::Vertex, u16>> = Vec::with_capacity(num_agents);
    for g in &goals {
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
        println!(
            "pop: [f-val: {:2}, g-val: {:2}, h-val: {:2}, num_col: {:2}]",
            cur_node.g_val + cur_node.h_val,
            cur_node.g_val,
            cur_node.h_val,
            cur_node.collisions.len()
        );
        if cur_node.collisions.is_empty() {
            // Solution found
            let elapsed_time = now.elapsed();
            println!("CPU time: {:?}", elapsed_time);
            println!("Mdd time: {:?}", mdd_time);
            println!("Car time: {:?}", col_time);
            println!("Cost: {}", cur_node.g_val);
            println!("Nodes expanded:  {}", pop_counter);
            println!("Nodes generated: {}", push_counter);
            return Some(cur_node.paths);
        }
        // TODO: Lazy heuristics

        // TODO: Meta-agent

        // Improved cbs: Always split on cardinal or semi-cardinal conflicts, then
        // attempt to bypass when there are no more cardinal and semi-cardinal conflicts.
        let col_now = Instant::now();
        detect_cardinal_conflicts(&mut cur_node);
        col_time += col_now.elapsed();
        let mut cur_collision: collision::Collision = cur_node.collisions[0];
        let mut attempt_bypass = true;
        for collision in &cur_node.collisions {
            match collision.conflict {
                cardinal::Cardinal::Full => {
                    // TODO: How do you increase the cost of both agents?
                    cur_collision = *collision;
                    attempt_bypass = false;
                    break;
                }
                cardinal::Cardinal::Semi => {
                    cur_collision = *collision;
                    attempt_bypass = false;
                    break;
                }
                cardinal::Cardinal::Non => {}
            }
        }
        if attempt_bypass {
            match bypass_collisions(&mut cur_node, map, &starts, &goals, &h_values) {
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
                .filter(|c| c.agent == constraint_agent as u8)
                .copied()
                .collect();
            let mut new_paths = cur_node.paths.clone();
            let min_path_length = match new_constraint.conflict {
                cardinal::Cardinal::Full => new_paths[constraint_agent].len(),
                cardinal::Cardinal::Semi => {
                    // Force the agent to increase its path length by 1 in cases where constraint
                    // is negative. In positive cases the agent path length will stay the same.
                    if !new_constraint.is_positive {
                        new_paths[constraint_agent].len()
                    } else {
                        new_paths[constraint_agent].len() - 1
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
pub struct Node {
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
