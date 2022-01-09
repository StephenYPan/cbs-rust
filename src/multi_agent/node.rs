use crate::datatype::{constraint::Constraint, vertex::Vertex};
use crate::multi_agent::collision::Collision;
use std::cmp::Ordering;

#[derive(Debug, Eq)]
pub struct Node {
    pub g_val: u16,
    pub h_val: u16,
    pub paths: Vec<Vec<Vertex>>,
    pub constraints: Vec<Constraint>,
    pub collisions: Vec<Collision>,
    // pub mdds: Vec<MDD>,
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

impl Node {
    fn new(
        g_val: u16,
        h_val: u16,
        paths: Vec<Vec<Vertex>>,
        constraints: Vec<Constraint>,
        collisions: Vec<Collision>,
    ) -> Node {
        Node {
            g_val: g_val,
            h_val: h_val,
            paths: paths,
            constraints: constraints,
            collisions: collisions,
        }
    }
}
