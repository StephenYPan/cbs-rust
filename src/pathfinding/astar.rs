use crate::datatype::vertex::Vertex;
use crate::pathfinding::lib::{get_next_loc, is_invalid_loc};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Find the shortest path from start to goal such that it satisfies the given constraints.
pub fn astar(
    map: &Vec<Vec<u8>>,
    start_loc: Vertex,
    goal_loc: Vertex,
    h_values: &HashMap<Vertex, u16>,
) -> Option<Vec<Vertex>> {
    let map_size: usize = map
        .iter()
        .flat_map(|v| v.iter())
        .filter(|&x| *x == 1)
        .count();
    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    let mut closed_list: HashMap<(Vertex, u16), usize> = HashMap::with_capacity(map_size);
    let mut tree = Tree::new();

    let root_idx = tree.add_node(start_loc, 0, *h_values.get(&start_loc).unwrap(), 0);
    let root = tree.tree[root_idx];
    closed_list.insert((root.loc, root.g_val), root_idx);
    open_list.push(root);

    while !open_list.is_empty() {
        let cur_node = open_list.pop().unwrap();
        let cur_idx = *closed_list.get(&(cur_node.loc, cur_node.g_val)).unwrap();
        if cur_node.loc == goal_loc {
            return Some(tree.get_path(cur_idx));
        }
        for action in 0..5 {
            let next_loc = match get_next_loc(cur_node.loc, action) {
                Some(vertex) => vertex,
                None => continue,
            };
            if is_invalid_loc(&map, next_loc) {
                continue;
            }
            // TODO: add constraints
            let new_idx = tree.add_node(
                next_loc,
                cur_node.g_val + 1,
                *h_values.get(&next_loc).unwrap(),
                cur_idx,
            );
            let new_node = tree.tree[new_idx];
            let new_key = (new_node.loc, new_node.g_val);
            match closed_list.get(&new_key) {
                Some(old_idx) => {
                    // Update existing node if it is a shorter path
                    if new_node < tree.tree[*old_idx] {
                        // Update key, guard against the key possibly not being set
                        let val = closed_list.entry(new_key).or_insert(new_idx);
                        *val = new_idx;
                        open_list.push(new_node)
                    }
                }
                None => {
                    closed_list.insert(new_key, new_idx);
                    open_list.push(new_node);
                }
            }
        }
    }
    None
}

/// source: https://dev.to/deciduously/no-more-tears-no-more-knots-arena-allocated-trees-in-rust-44k6
struct Tree {
    tree: Vec<Node>,
    parent_node: Vec<usize>,
}

impl Tree {
    fn new() -> Tree {
        Tree {
            tree: Vec::new(),
            parent_node: Vec::new(),
        }
    }

    fn add_node(&mut self, loc: Vertex, g_val: u16, h_val: u16, parent: usize) -> usize {
        // If node exists, then update and return its index
        for (i, n) in self.tree.iter_mut().enumerate() {
            if n.g_val == g_val && n.loc == loc {
                self.parent_node[i] = parent;
                return i;
            }
        }
        // Add new node and return its index
        let idx = self.tree.len();
        self.tree.push(Node::new(loc, g_val, h_val));
        self.parent_node.push(parent);
        idx
    }

    /// Runtime is O(c) where c is the path length.
    fn get_path(&self, goal_idx: usize) -> Vec<Vertex> {
        // Travel backwards from the goal node to the start node.
        let mut path: Vec<Vertex> = Vec::new();
        path.push(self.tree[goal_idx].loc);
        let mut next_idx = self.parent_node[goal_idx];
        while next_idx != 0 {
            path.push(self.tree[next_idx].loc);
            next_idx = self.parent_node[next_idx];
        }
        path.push(self.tree[next_idx].loc); // Add start location
        path.reverse();
        path
    }
}

#[derive(Debug, Eq, Copy, Clone)]
struct Node {
    loc: Vertex,
    g_val: u16,
    h_val: u16,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.loc.eq(&other.loc) && self.g_val.eq(&other.g_val)
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

    fn lt(&self, other: &Self) -> bool {
        self.g_val + self.h_val < other.g_val + other.h_val
    }
}

impl Node {
    fn new(loc: Vertex, g_val: u16, h_val: u16) -> Node {
        Node {
            loc: loc,
            g_val: g_val,
            h_val: h_val,
        }
    }
}
