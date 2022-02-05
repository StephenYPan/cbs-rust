use crate::datatype::{cardinal, collision, constraint, edge, lib, vertex};
use crate::single_agent::dijkstra;
use std::cmp::min;
use std::collections::{hash_map::DefaultHasher, HashSet};
use std::hash::{Hash, Hasher};

use cached::proc_macro::cached;
use cached::SizedCache;
// use rayon::prelude::*;

// TODO: Add hash value to mdd
#[derive(Debug, Eq, Clone)]
pub struct Mdd {
    pub mdd: Vec<Vec<edge::Edge>>,
}

// TODO: Change to compare hash
impl PartialEq for Mdd {
    fn eq(&self, other: &Self) -> bool {
        if self.mdd.len() != other.mdd.len() {
            false
        } else {
            for (i, layer) in self.mdd.iter().enumerate() {
                for (e1, e2) in layer.iter().zip(&other.mdd[i]) {
                    if e1 != e2 {
                        return false;
                    }
                }
            }
            true
        }
    }
}

impl Mdd {
    pub fn new(
        map: &[Vec<u8>],
        path: &[vertex::Vertex],
        constraints: &[constraint::Constraint],
    ) -> Mdd {
        let mdd = build_mdd(map, path, constraints);
        // Mdd must contain the path edges.
        for (timestep, edge) in path.iter().zip(&path[1..]).enumerate() {
            assert!(
                &mdd[timestep]
                    .iter()
                    .any(|e| e.0 == *edge.0 && e.1 == *edge.1),
                "path edge: {:?} at t={} not in mdd[{}]: {:?}",
                edge,
                timestep,
                timestep,
                mdd[timestep]
            );
        }
        Mdd { mdd }
    }
}

const CARDINAL_CACHE: usize = 1000;

#[cached(
    type = "SizedCache<u64, Option<collision::Collision>>",
    create = "{ SizedCache::with_size(CARDINAL_CACHE) }",
    convert = "{ _hash }",
    sync_writes = true
)]
pub fn find_cardinal_conflict(
    mdd1: &Mdd,
    mdd2: &Mdd,
    agent1: u8,
    agent2: u8,
    _hash: u64,
) -> Option<collision::Collision> {
    // Find the first cardinal conflicts and return it.
    let min_timestep = min(mdd1.mdd.len(), mdd2.mdd.len());
    for i in 0..min_timestep {
        let layer1 = &mdd1.mdd[i];
        let layer2 = &mdd2.mdd[i];
        let edge1 = layer1[0];
        let edge2 = layer2[0];
        // Check for edge collision, compare first edge with reversed second edge
        if layer1.len() == 1 && layer2.len() == 1 && edge1.0 == edge2.1 && edge1.1 == edge2.0 {
            return Some(collision::Collision::new(
                agent1,
                agent2,
                constraint::Location::new(edge1),
                (i + 1) as u16,
                cardinal::Cardinal::Full,
            ));
        }
        let vertex1: HashSet<vertex::Vertex> = layer1.iter().map(|edge| edge.1).collect();
        let vertex2: HashSet<vertex::Vertex> = layer2.iter().map(|edge| edge.1).collect();
        if vertex1.len() == 1 && vertex2.len() == 1 && vertex1 == vertex2 {
            return Some(collision::Collision::new(
                agent1,
                agent2,
                constraint::Location::new(layer1[0].1),
                (i + 1) as u16,
                cardinal::Cardinal::Full,
            ));
        }
    }
    find_extended_mdd_conflict(mdd1, mdd2, agent1, agent2)
}

#[cached(
    type = "SizedCache<u64, Option<collision::Collision>>",
    create = "{ SizedCache::with_size(CARDINAL_CACHE) }",
    convert = "{ _hash }",
    sync_writes = true
)]
pub fn find_dependency_conflict(
    mdd1: &Mdd,
    mdd2: &Mdd,
    agent1: u8,
    agent2: u8,
    _hash: u64,
) -> Option<collision::Collision> {
    // Find all the dependency conflicts return the last one.
    let mut joint_mdd: HashSet<(usize, vertex::Vertex, vertex::Vertex)> = HashSet::new();
    joint_mdd.insert((0, mdd1.mdd[0][0].0, mdd2.mdd[0][0].0));
    let mut dependency_conflict = constraint::Location::default();
    let min_timestep = min(mdd1.mdd.len(), mdd2.mdd.len());
    for i in 0..min_timestep {
        let layer1 = &mdd1.mdd[i];
        let layer2 = &mdd2.mdd[i];
        let mut is_dependent = true;
        for edge1 in layer1 {
            for edge2 in layer2 {
                if joint_mdd.get(&(i, edge1.0, edge2.0)).is_none() {
                    continue;
                }
                if edge1.1 == edge2.1 {
                    // Vertex dependency conflict
                    dependency_conflict = constraint::Location::new(edge1.1);
                    continue;
                }
                if edge1.0 == edge2.1 && edge1.1 == edge2.0 {
                    // Edge dependency conflict
                    dependency_conflict = constraint::Location::new(*edge1);
                    continue;
                }
                joint_mdd.insert((i + 1, edge1.1, edge2.1));
                is_dependent = false;
            }
        }
        if is_dependent {
            return Some(collision::Collision::new(
                agent1,
                agent2,
                dependency_conflict,
                (i + 1) as u16,
                cardinal::Cardinal::Semi,
            ));
        }
    }
    find_extended_mdd_conflict(mdd1, mdd2, agent1, agent2)
}

fn find_extended_mdd_conflict(
    mdd1: &Mdd,
    mdd2: &Mdd,
    agent1: u8,
    agent2: u8,
) -> Option<collision::Collision> {
    // Find the first cardinal conflicts and return it.
    if mdd1.mdd.len() == mdd2.mdd.len() {
        return None;
    }
    let start_time = min(mdd1.mdd.len(), mdd2.mdd.len());
    let mdd;
    let other_vertex: vertex::Vertex;
    if mdd1.mdd.len() > mdd2.mdd.len() {
        mdd = mdd1;
        other_vertex = mdd2.mdd[start_time - 1].last().unwrap().1;
    } else {
        mdd = mdd2;
        other_vertex = mdd1.mdd[start_time - 1].last().unwrap().1;
    }
    for (i, v) in mdd.mdd[start_time..].iter().enumerate() {
        if v.len() == 1 && v[0].1 == other_vertex {
            return Some(collision::Collision::new(
                agent1,
                agent2,
                constraint::Location::new(other_vertex),
                (start_time + i + 1) as u16,
                cardinal::Cardinal::Full,
            ));
        }
    }
    None
}

const MDD_CACHE_SIZE: usize = 1000;

#[cached(
    type = "SizedCache<(vertex::Vertex, usize, u64), Vec<Vec<edge::Edge>>>",
    create = "{ SizedCache::with_size(MDD_CACHE_SIZE) }",
    convert = "{ (path[0], path.len(), lib::hash(constraints)) }",
    sync_writes = true
)]
fn build_mdd(
    map: &[Vec<u8>],
    path: &[vertex::Vertex],
    constraints: &[constraint::Constraint],
) -> Vec<Vec<edge::Edge>> {
    let mut mdd = Vec::with_capacity(path.len() - 1);
    // use positive to generate partial mdd
    let path_len = path.len() as u16;
    let mut pos_constraints: HashSet<(u16, vertex::Vertex)> = constraints
        .iter()
        .filter(|c| c.is_positive && c.timestep < path_len)
        .flat_map(|c| match c.is_edge {
            true => vec![(c.timestep - 1, c.loc.0), (c.timestep, c.loc.1)],
            false => vec![(c.timestep, c.loc.1)],
        })
        .collect();
    pos_constraints.insert((0, path[0]));
    pos_constraints.insert((path_len - 1, *path.last().unwrap()));
    let mut intermediate_goals: Vec<(u16, vertex::Vertex)> =
        pos_constraints.iter().copied().collect();
    intermediate_goals.sort_by_key(|c| c.0);
    // Generate partial mdds using the intermediate goal nodes
    for (start, goal) in intermediate_goals.iter().zip(&intermediate_goals[1..]) {
        // return value of build_partial_mdd extended by mdd to maintain order of edges.
        let max_cost = goal.0 - start.0;
        let partial_mdd: Vec<Vec<edge::Edge>> = build_partial_mdd(map, start.1, goal.1, max_cost);
        mdd.extend(partial_mdd); // partial_mdd is moved and cannot be referenced again.
    }
    // use negative to filter the mdd
    let neg_constraints: Vec<(u16, constraint::Location)> = constraints
        .iter()
        .filter(|c| !c.is_positive)
        .map(|c| match c.is_edge {
            true => (c.timestep, constraint::Location::new(c.loc)),
            false => (c.timestep, constraint::Location::new(c.loc.1)),
        })
        .collect();
    // Remove edges from the mdd according to each negative edge.
    if neg_constraints.is_empty() {
        // Early exit.
        return mdd;
    }
    for (timestep, constraint) in neg_constraints {
        match constraint {
            constraint::Location::Edge(edge) => {
                // Remove single edge
                mdd[(timestep - 1) as usize].retain(|e| *e != edge)
            }
            constraint::Location::Vertex(vertex) => {
                // Remove all edges going to vertex
                mdd[(timestep - 1) as usize].retain(|e| e.1 != vertex);
            }
        }
    }
    // Remove all the edges that do not generate a valid path by doing a forward and backward pass.
    let mdd_length = path_len - 1;
    for timestep in (1..mdd_length).rev() {
        // Remove backwards, nodes without children
        let cur_layer: HashSet<vertex::Vertex> =
            mdd[timestep as usize].iter().map(|edge| edge.0).collect();
        let prev_layer = &mdd[(timestep - 1) as usize];

        let mut edge_position: Vec<usize> = vec![];
        for (edge_index, edge) in prev_layer.iter().enumerate().rev() {
            match cur_layer.contains(&edge.1) {
                true => {}
                false => edge_position.push(edge_index),
            }
        }
        for edge in edge_position {
            mdd[(timestep - 1) as usize].remove(edge);
        }
    }
    for timestep in 0..(mdd_length - 1) {
        // Remove forward, nodes without parents
        let cur_layer: HashSet<vertex::Vertex> =
            mdd[timestep as usize].iter().map(|edge| edge.1).collect();
        let next_layer = &mdd[(timestep + 1) as usize];

        let mut edge_position: Vec<usize> = vec![];
        for (edge_index, edge) in next_layer.iter().enumerate().rev() {
            match cur_layer.contains(&edge.0) {
                true => {}
                false => edge_position.push(edge_index),
            }
        }
        for edge in edge_position {
            mdd[(timestep + 1) as usize].remove(edge);
        }
    }
    for v in &mut mdd {
        v.shrink_to_fit();
    }
    mdd.shrink_to_fit();
    mdd
}

const PARTIAL_MDD_CACHE_SIZE: usize = 1000;

#[cached(
    type = "SizedCache<(vertex::Vertex, vertex::Vertex, u16), Vec<Vec<edge::Edge>>>",
    create = "{ SizedCache::with_size(PARTIAL_MDD_CACHE_SIZE) }",
    convert = "{ (start, goal, max_cost) }",
    sync_writes = true
)]
fn build_partial_mdd(
    map: &[Vec<u8>],
    start: vertex::Vertex,
    goal: vertex::Vertex,
    max_cost: u16,
) -> Vec<Vec<edge::Edge>> {
    let mut partial_mdd = Vec::with_capacity(max_cost as usize);
    let start_h_val = dijkstra::compute_heuristics(map, start);
    let goal_h_val = dijkstra::compute_heuristics(map, goal);
    let valid_locations: Vec<(vertex::Vertex, u16)> = start_h_val
        .iter() // parallelize iter(?)
        .filter(|(k, v)| *v + goal_h_val[k] <= max_cost)
        .map(|(k, v)| (*k, *v))
        .collect();
    for t in 0..max_cost {
        let mut partial_mdd_timestep_t: Vec<edge::Edge> = Vec::new();
        let start_locations: Vec<vertex::Vertex> = valid_locations
            .iter()
            .filter(|(v, h_val)| *h_val <= t && t + goal_h_val[v] <= max_cost)
            .map(|(v, _)| *v)
            .collect();
        for cur_loc in start_locations {
            for action in 0..5 {
                let next_loc = match dijkstra::get_next_loc(map, cur_loc, action) {
                    Some(vertex) => vertex,
                    None => continue,
                };
                if t + 1 + goal_h_val[&next_loc] > max_cost {
                    continue;
                }
                partial_mdd_timestep_t.push(edge::Edge(cur_loc, next_loc));
            }
        }
        partial_mdd_timestep_t.shrink_to_fit();
        partial_mdd.push(partial_mdd_timestep_t);
    }
    partial_mdd
}
