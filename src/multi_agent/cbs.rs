#![allow(unused)]
use crate::datatype::{constraint::Constraint, constraint::Location, edge::Edge, vertex::Vertex};
use crate::single_agent::{astar::astar, dijkstra::compute_heuristics};
use std::collections::{BinaryHeap, HashMap, HashSet};

// def get_sum_of_cost(paths):
//     rst = 0
//     for path in paths:
//         rst += len(path) - 1
//     return rst

// def get_location(path, time):
//     return path[max(0, time)] if time < len(path) else path[-1]

pub fn cbs(map: &Vec<Vec<u8>>, starts: Vec<Vertex>, goals: Vec<Vertex>) {}