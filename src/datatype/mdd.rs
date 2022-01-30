#![allow(unused)]
use crate::datatype::{collision, constraint, edge, vertex};
use crate::single_agent::dijkstra;
use std::collections::{hash_map::DefaultHasher, HashSet};
use std::fmt;
use std::hash::{Hash, Hasher};

use cached::proc_macro::cached;
use cached::SizedCache;
use rayon::prelude::*;

const MDD_CACHE_SIZE: usize = 1000;
const PARTIAL_MDD_CACHE_SIZE: usize = 1000;

pub struct Mdd {
    pub mdd: Vec<Vec<edge::Edge>>,
}

impl Mdd {
    pub fn new(
        map: &[Vec<u8>],
        path: &[vertex::Vertex],
        constraints: &[constraint::Constraint],
    ) -> Mdd {
        let mut state = DefaultHasher::new();
        constraints.hash(&mut state);
        let hash = state.finish();
        let mdd = build_mdd(map, path, constraints, path[0], hash);
        Mdd { mdd }
    }

    pub fn find_cardinal_conflict(&self, other: &Self) -> Option<collision::Collision> {
        None
    }

    pub fn find_dependency_conflict(&self, other: &Self) -> Option<collision::Collision> {
        None
    }

    fn find_extended_mdd_conflict(&self, other: &Self) -> Option<collision::Collision> {
        None
    }
}

#[cached(
    type = "SizedCache<(vertex::Vertex, u64), Vec<Vec<edge::Edge>>>",
    create = "{ SizedCache::with_size(MDD_CACHE_SIZE) }",
    convert = "{ (_start_loc, _hash) }",
    sync_writes = true
)]
fn build_mdd(
    map: &[Vec<u8>],
    path: &[vertex::Vertex],
    constraints: &[constraint::Constraint],
    _start_loc: vertex::Vertex,
    _hash: u64,
) -> Vec<Vec<edge::Edge>> {
    let mut mdd = Vec::with_capacity(path.len() - 1);
    // use positive to generate partial mdd
    let path_len = path.len() as u16;
    let mut pos_constraints: Vec<(u16, vertex::Vertex)> = constraints
        .iter()
        .filter(|c| c.is_positive && c.timestep < path_len)
        .flat_map(|c| match c.is_edge {
            true => vec![(c.timestep - 1, c.loc.0), (c.timestep, c.loc.1)],
            false => vec![(c.timestep, c.loc.1)],
        })
        .collect();
    pos_constraints.push((0, path[0]));
    pos_constraints.push((path_len - 1, *path.last().unwrap()));
    pos_constraints.sort_by_key(|c| c.0);
    // Generate partial mdds using the intermediate goal nodes
    for (start, goal) in pos_constraints.iter().zip(&pos_constraints[1..]) {
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
                // Remove all edges going to and from vertex
                mdd[(timestep - 1) as usize].retain(|e| e.1 != vertex);
                mdd[timestep as usize].retain(|e| e.0 != vertex);
            }
        }
    }
    // Remove all the edges that do not generate a valid path by doing a forward and backward pass.
    let mdd_length = path_len - 1;
    for timestep in (mdd_length - 1)..1 {
        // Remove backwards, nodes without children
        let cur_layer: HashSet<vertex::Vertex> =
            mdd[timestep as usize].iter().map(|edge| edge.0).collect();
        let prev_layer = &mdd[(timestep - 1) as usize];
        let prev_layer_size = prev_layer.len() - 1;
        let mut edge_position: Vec<usize> = vec![];
        for (i, edge) in prev_layer.iter().rev().enumerate() {
            match cur_layer.contains(&edge.1) {
                true => {}
                false => edge_position.push(prev_layer_size - i),
            }
        }
        for position in edge_position {
            mdd[(timestep + 1) as usize].remove(position);
        }
    }
    for timestep in 1..(mdd_length - 1) {
        // Remove forward, nodes without parents
        let cur_layer: HashSet<vertex::Vertex> =
            mdd[timestep as usize].iter().map(|edge| edge.1).collect();
        let next_layer = &mdd[(timestep + 1) as usize];
        let next_layer_size = next_layer.len() - 1;
        let mut edge_position: Vec<usize> = vec![];
        for (i, edge) in next_layer.iter().rev().enumerate() {
            match cur_layer.contains(&edge.0) {
                true => {}
                false => edge_position.push(next_layer_size - i),
            }
        }
        for position in edge_position {
            mdd[(timestep + 1) as usize].remove(position);
        }
    }
    // assert the mdd contains the path edges
    for (timestep, edge) in path.iter().zip(&path[1..]).enumerate() {
        assert!(
            &mdd[timestep]
                .iter()
                .any(|e| e.0 == *edge.0 && e.1 == *edge.1),
            "path edge: {:?} at t={} not in mdd: {:?}",
            edge,
            timestep,
            mdd[timestep]
        );
    }
    for v in &mut mdd {
        v.shrink_to_fit();
    }
    mdd.shrink_to_fit();
    mdd
}

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
        .par_iter() // parallelize iter
        .filter(|(k, v)| **v + *goal_h_val.get(k).unwrap() <= max_cost)
        .map(|(k, v)| (*k, *v))
        .collect();
    for t in 0..max_cost {
        let mut partial_mdd_timestep_t: Vec<edge::Edge> = Vec::new();
        let start_locations: Vec<vertex::Vertex> = valid_locations
            .iter()
            .filter(|(v, h_val)| *h_val <= t && t + *goal_h_val.get(v).unwrap() <= max_cost)
            .map(|(v, _)| *v)
            .collect();
        for cur_loc in start_locations {
            for action in 0..5 {
                let next_loc = match dijkstra::get_next_loc(map, cur_loc, action) {
                    Some(vertex) => vertex,
                    None => continue,
                };
                if t + 1 + *goal_h_val.get(&next_loc).unwrap() > max_cost {
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
