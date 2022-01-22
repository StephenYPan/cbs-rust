#![allow(unused)]
use crate::datatype::{collision, constraint, edge, vertex};
use crate::single_agent::dijkstra;
use std::collections::hash_map::DefaultHasher;
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
        let mdd = build_mdd(map, path, constraints, hash);
        // assert the mdd contains the path edges
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
    type = "SizedCache<u64, Vec<Vec<edge::Edge>>>",
    create = "{ SizedCache::with_size(MDD_CACHE_SIZE) }",
    convert = "{ _hash }",
    sync_writes = true
)]
fn build_mdd(
    map: &[Vec<u8>],
    path: &[vertex::Vertex],
    constraints: &[constraint::Constraint],
    _hash: u64,
) -> Vec<Vec<edge::Edge>> {
    let mut mdd = Vec::with_capacity(path.len() - 1);
    // return value of build_partial_mdd extended by mdd to maintain order of edges.
    // mdd.extend(partial_mdd); // partial_mdd is moved and cannot be referenced again.
    // separate into positive and negative constraints
    // use positive to generate partial mdd
    // use negative to filter the mdd

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
