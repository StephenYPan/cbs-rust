use crate::datatype::vertex::Vertex;
use crate::pathfinding::lib::{get_next_loc, is_invalid_loc};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

pub fn astar(
    map: &Vec<Vec<u8>>,
    start_loc: Vertex,
    goal_loc: Vertex,
    h_values: &HashMap<Vertex, u16>,
) -> Option<Vec<Vertex>> {
    let map_size: usize = map.iter().map(Vec::len).sum();
    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    let mut closed_list: HashMap<(Vertex, u16), Node> = HashMap::with_capacity(map_size);
    let mut tree = Tree { tree: Vec::new() };
    let root_idx = tree.add_node(
        start_loc,
        0,
        h_values.get(&start_loc).unwrap().clone(),
        0,
        None,
    );
    let root = tree.tree[root_idx];
    open_list.push(root);
    closed_list.insert((root.loc, root.timestep), root);
    while open_list.len() > 0 {
        let cur_node = open_list.pop().unwrap();
        if cur_node.loc == goal_loc {
            return Some(tree.get_path(cur_node.idx));
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
            let idx = tree.add_node(
                next_loc,
                cur_node.g_val + 1,
                h_values.get(&next_loc).unwrap().clone(),
                cur_node.timestep + 1,
                Some(cur_node.idx),
            );
            let new_node = tree.tree[idx];
            let key = (new_node.loc, new_node.timestep);
            match closed_list.get(&key) {
                Some(node) => {
                    // Update existing node if it is a shorter path
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
    None
}

/// source: https://dev.to/deciduously/no-more-tears-no-more-knots-arena-allocated-trees-in-rust-44k6
#[derive(Debug, Default)]
struct Tree {
    tree: Vec<Node>,
}

impl Tree {
    fn add_node(
        &mut self,
        loc: Vertex,
        g_val: u16,
        h_val: u16,
        timestep: u16,
        parent: Option<usize>,
    ) -> usize {
        // Check if node exists, update, and return the index
        for n in &mut self.tree {
            if n.loc == loc && n.timestep == timestep {
                n.g_val = g_val;
                n.h_val = h_val;
                n.parent = parent;
                return n.idx;
            }
        }
        // Add new node and return the index
        let idx = self.tree.len();
        let node = Node::new(idx, loc, g_val, h_val, timestep, parent);
        self.tree.push(node);
        idx
    }

    fn get_path(&self, idx: usize) -> Vec<Vertex> {
        // Travel backwards from a index to the root
        let goal_node = self.tree[idx];
        let mut path: Vec<Vertex> = Vec::new();
        path.push(goal_node.loc);
        let mut optional = goal_node.parent;
        while let Some(next_idx) = optional {
            let next_node = self.tree[next_idx];
            path.push(next_node.loc);
            optional = next_node.parent;
        }
        path.reverse();
        path
    }
}

#[derive(Debug, Eq, Copy, Clone)]
struct Node {
    idx: usize,
    loc: Vertex,
    g_val: u16,
    h_val: u16,
    timestep: u16,
    parent: Option<usize>,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.loc.eq(&other.loc) && self.timestep.eq(&other.timestep)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        let f_value = self.g_val + self.h_val;
        let other_f_value = other.g_val + other.h_val;
        f_value.cmp(&other_f_value).reverse()
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
    fn new(
        idx: usize,
        loc: Vertex,
        g_val: u16,
        h_val: u16,
        timestep: u16,
        parent: Option<usize>,
    ) -> Node {
        Node {
            idx: idx,
            loc: loc,
            g_val: g_val,
            h_val: h_val,
            timestep: timestep,
            parent: parent,
        }
    }
}
