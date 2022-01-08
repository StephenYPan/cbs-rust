use crate::datatype::vertex::Vertex;
use crate::pathfinding::lib::{get_next_loc, is_invalid_loc};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Use Dijkstra to build a shortest path from a location to all other locations
pub fn compute_heuristics(map: &Vec<Vec<u8>>, start_loc: Vertex) -> HashMap<Vertex, u16> {
    let map_size: usize = map
        .iter()
        .flat_map(|v| v.iter())
        .filter(|&x| *x == 1)
        .count();
    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    let mut closed_list: HashMap<Vertex, Node> = HashMap::with_capacity(map_size);

    let root = Node::new(start_loc, 0);
    closed_list.insert(root.loc, root);
    open_list.push(root);

    while !open_list.is_empty() {
        let cur_node = open_list.pop().unwrap();
        for action in 0..4 {
            let next_loc = match get_next_loc(cur_node.loc, action) {
                Some(vertex) => vertex,
                None => continue,
            };
            if is_invalid_loc(&map, next_loc) {
                continue;
            }
            let new_node = Node::new(next_loc, cur_node.g_val + 1);
            let key = new_node.loc;
            match closed_list.get(&key) {
                Some(_) => {
                    // The if condition will never be true because the action cost are uniform.
                    // // Update existing node if it is a shorter path.
                    // if new_node < *node {
                    //     // Update key, guard against the key possibly not being set
                    //     let val = closed_list.entry(key).or_insert(new_node);
                    //     *val = new_node;
                    //     open_list.push(new_node);
                    // }
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
    let mut h_values: HashMap<Vertex, u16> = HashMap::with_capacity(closed_list.len());
    for (vertex, node) in closed_list {
        h_values.insert(vertex, node.g_val);
    }
    h_values
}

#[derive(Debug, Eq, Copy, Clone)]
struct Node {
    loc: Vertex,
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
    fn new(loc: Vertex, g_val: u16) -> Node {
        Node {
            loc: loc,
            g_val: g_val,
        }
    }
}
