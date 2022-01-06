use crate::datatype::vertex::Vertex;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

const DIRECTION: [(i32, i32); 4] = [
    (0, -1), // Left
    (1, 0),  // Up
    (0, 1),  // Right
    (-1, 0), // Down
];

/// Size of tuple is (i32, i32) to avoid the edge case where
/// units in the Vertex (u16, u16) is between 2^15 and 2^16-1.
fn get_next_loc(loc: Vertex, action: usize) -> (i32, i32) {
    let dir = DIRECTION[action];
    (loc.0 as i32 + dir.0, loc.1 as i32 + dir.1)
}

fn is_invalid_loc(map: &Vec<Vec<u8>>, next_loc: (i32, i32)) -> bool {
    let height = map.len() as i32;
    let width = map[0].len() as i32;
    next_loc.0 < 0 || next_loc.0 >= height // Out of bound in vertical axis
        || next_loc.1 < 0 || next_loc.1 >= width // Out of bound in horizontal axis
        || map[next_loc.0 as usize][next_loc.1 as usize] == 0 // Map obstacle, will never panic
}

/// Use Dijkstra to build a shortest path from a location to all other locations
pub fn compute_heuristics(map: &Vec<Vec<u8>>, start_loc: Vertex) -> HashMap<Vertex, u16> {
    let map_size: usize = map.iter().map(Vec::len).sum();
    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    let mut closed_list: HashMap<Vertex, Node> = HashMap::with_capacity(map_size);
    let root = Node::new(start_loc, 0);
    open_list.push(root);
    closed_list.insert(root.loc, root);
    while open_list.len() > 0 {
        let cur_node = open_list.pop().unwrap();
        for action in 0..4 {
            let next_loc = get_next_loc(cur_node.loc, action);
            if is_invalid_loc(&map, next_loc) {
                continue;
            }
            let new_node = Node::new(
                Vertex(next_loc.0 as u16, next_loc.1 as u16),
                cur_node.g_val + 1,
            );
            match closed_list.get(&new_node.loc) {
                Some(node) => {
                    // Update existing node if it is a shorter path
                    if new_node.g_val < node.g_val {
                        // Update key, guard against the key possibly not being set
                        let val = closed_list.entry(new_node.loc).or_insert(new_node);
                        *val = new_node;
                        open_list.push(new_node);
                    }
                }
                None => {
                    closed_list.insert(new_node.loc, new_node);
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

impl PartialEq<Vertex> for Node {
    fn eq(&self, other: &Vertex) -> bool {
        self.loc == *other
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
}

impl Node {
    fn new(loc: Vertex, g_val: u16) -> Node {
        Node {
            loc: loc,
            g_val: g_val,
        }
    }
}
