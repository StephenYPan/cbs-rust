#![allow(unused)]
use crate::datatype::{constraint::Constraint, constraint::Location, edge::Edge, vertex::Vertex};
use crate::single_agent::{astar::astar, dijkstra::compute_heuristics};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::fmt;

#[derive(Debug, Eq)]
struct Node {
    g_val: u16,
    h_val: u16,
    paths: Vec<Vec<Vertex>>,
    constraints: Vec<Constraint>,
    collisions: Vec<Collision>,
    // mdds: Vec<MDD>,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        if self.g_val.eq(&other.g_val) {
            let paths: Vec<&Vertex> = self.paths.iter().flat_map(|v| v.iter()).clone().collect();
            let other_paths: Vec<&Vertex> = other.paths.iter().flat_map(|v| v.iter()).collect();
            for (v1, v2) in paths.iter().zip(other_paths.iter()) {
                if v1 != v2 {
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

#[derive(Eq)]
struct Collision {
    a1: u8,
    a2: u8,
    loc: Edge,
    is_edge: bool,
    timestep: u16,
}

impl PartialEq for Collision {
    fn eq(&self, other: &Self) -> bool {
        self.loc.eq(&other.loc) && self.is_edge & other.is_edge && self.timestep == other.timestep
    }
}

impl fmt::Debug for Collision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let loc: Location = if self.is_edge {
            Location::new(self.loc)
        } else {
            Location::new(self.loc.1)
        };
        f.debug_struct("Constraint")
            .field("a1", &self.a1)
            .field("a2", &self.a2)
            .field("loc", &loc)
            .field("timestep", &self.timestep)
            .finish()
    }
}
