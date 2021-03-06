use crate::datatype::vertex;
use crate::single_agent::lib;
use std::collections::{BinaryHeap, HashMap};

use cached::proc_macro::cached;
use cached::SizedCache;

const CACHE_SIZE: usize = 100;

/// Use Dijkstra to build a shortest path from a location to all other locations
#[cached(
    type = "SizedCache<vertex::Vertex, HashMap<vertex::Vertex, u16>>",
    create = "{ SizedCache::with_size(CACHE_SIZE) }",
    convert = "{ location }",
    sync_writes = true
)]
pub fn compute_heuristics(
    map: &[Vec<u8>],
    location: vertex::Vertex,
) -> HashMap<vertex::Vertex, u16> {
    let map_size: usize = map
        .iter()
        .flat_map(|v| v.iter())
        .filter(|&x| *x == 1)
        .count();
    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    let mut closed_list: HashMap<vertex::Vertex, Node> = HashMap::with_capacity(map_size);

    let root = Node::new(location, 0);
    closed_list.insert(root.loc, root);
    open_list.push(root);

    while !open_list.is_empty() {
        let cur_node = open_list.pop().unwrap();
        for action in 0..4 {
            let next_loc = match lib::get_next_loc(map, cur_node.loc, action) {
                Some(vertex) => vertex,
                None => continue,
            };
            let new_node = Node::new(next_loc, cur_node.g_val + 1);
            let key = new_node.loc;
            match closed_list.get(&key) {
                Some(node) => {
                    // The if condition will never be true because the action cost are uniform.
                    // // Update existing node if it is a shorter path.
                    if new_node < *node {
                        // Update key, guard against the key possibly not being set
                        let val = closed_list.entry(key).or_insert(new_node);
                        *val = new_node;
                        open_list.push(new_node);
                    }
                }
                None => {
                    closed_list.insert(key, new_node);
                    open_list.push(new_node);
                }
            }
        }
    }
    // Build the heuristic table starting from start_loc.
    // Capacity is exactly the length of closed_list.
    let mut h_values: HashMap<vertex::Vertex, u16> = HashMap::with_capacity(closed_list.len());
    for (vertex, node) in closed_list {
        h_values.insert(vertex, node.g_val);
    }
    h_values
}

use std::cmp::Ordering;

#[derive(Debug, Eq, Copy, Clone)]
struct Node {
    loc: vertex::Vertex,
    g_val: u16,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.loc.eq(&other.loc)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.g_val.cmp(&other.g_val).reverse()
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }

    fn lt(&self, other: &Self) -> bool {
        self.g_val < other.g_val
    }

    fn le(&self, other: &Self) -> bool {
        self.g_val <= other.g_val
    }

    fn gt(&self, other: &Self) -> bool {
        self.g_val > other.g_val
    }

    fn ge(&self, other: &Self) -> bool {
        self.g_val >= other.g_val
    }
}

impl Node {
    fn new(loc: vertex::Vertex, g_val: u16) -> Node {
        Node { loc, g_val }
    }
}
