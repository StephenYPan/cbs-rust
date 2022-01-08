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
    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    let mut tree = Tree::new();

    let root_h_val = *h_values.get(&start_loc).unwrap();
    open_list.push(tree.add_node(start_loc, 0, root_h_val, 0, 0).unwrap());

    while !open_list.is_empty() {
        let cur_node = open_list.pop().unwrap();
        let cur_idx = tree.get_node_idx(cur_node);
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
            match tree.add_node(
                next_loc,
                cur_node.g_val + 1,
                *h_values.get(&next_loc).unwrap(),
                cur_node.timestep + 1,
                cur_idx,
            ) {
                Some(node) => open_list.push(node),
                None => continue,
            }
        }
    }
    None
}

/// source: https://dev.to/deciduously/no-more-tears-no-more-knots-arena-allocated-trees-in-rust-44k6
struct Tree {
    tree: Vec<Node>,
    parent_node: Vec<usize>,
    visited_node: HashMap<(Vertex, u16), usize>,
}

impl Tree {
    fn new() -> Tree {
        Tree {
            tree: Vec::new(),
            parent_node: Vec::new(),
            visited_node: HashMap::new(),
        }
    }

    /// If node exists and is a shorter path, then update and return the node, otherwise none.
    /// If node does not exists, then add a new node and return the node
    fn add_node(
        &mut self,
        loc: Vertex,
        g_val: u16,
        h_val: u16,
        timestep: u16,
        parent: usize,
    ) -> Option<Node> {
        match self.visited_node.get(&(loc, timestep)) {
            Some(&idx) => {
                let prev_f_val = self.tree[idx].g_val + self.tree[idx].h_val;
                let cur_f_val = g_val + h_val;
                if cur_f_val >= prev_f_val {
                    None
                } else {
                    self.tree[idx].g_val = g_val;
                    self.tree[idx].h_val = h_val;
                    self.parent_node[idx] = parent;
                    Some(self.tree[idx])
                }
            }
            None => {
                let node = Node::new(loc, g_val, h_val, timestep);
                let node_idx = self.tree.len();
                self.tree.push(Node::new(loc, g_val, h_val, timestep));
                self.parent_node.push(parent);
                self.visited_node.insert((loc, timestep), node_idx);
                Some(node)
            }
        }
    }

    fn get_node_idx(&self, node: Node) -> usize {
        *self.visited_node.get(&(node.loc, node.timestep)).unwrap()
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
    timestep: u16,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.timestep.eq(&other.timestep) && self.loc.eq(&other.loc)
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
    fn new(loc: Vertex, g_val: u16, h_val: u16, timestep: u16) -> Node {
        Node {
            loc: loc,
            g_val: g_val,
            h_val: h_val,
            timestep: timestep,
        }
    }
}
