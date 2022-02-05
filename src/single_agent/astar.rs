use crate::datatype::{constraint, edge, vertex};
use crate::single_agent::dijkstra;
use std::cmp::max;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Return true if the action is a negative constraint, otherwise false.
fn is_neg_constraint(
    cur_loc: vertex::Vertex,
    next_loc: vertex::Vertex,
    next_t: u16,
    neg_constraints: &HashSet<(edge::Edge, bool, u16)>,
) -> bool {
    // Checks hashset for edge then vertex.
    match neg_constraints.get(&(edge::Edge(cur_loc, next_loc), true, next_t)) {
        Some(_) => true,
        None => neg_constraints
            .get(&(edge::Edge(vertex::Vertex(0, 0), next_loc), false, next_t))
            .is_some(),
    }
}

/// Return true if the action is not a positive constraint, otherwise false.
fn is_pos_constraint(
    cur_loc: vertex::Vertex,
    next_loc: vertex::Vertex,
    next_t: u16,
    pos_constraints: &HashMap<(u16, bool), edge::Edge>,
) -> bool {
    // Check hashmap for edge then vertex.
    match pos_constraints.get(&(next_t, true)) {
        Some(&edge) => edge != edge::Edge(cur_loc, next_loc),
        None => match pos_constraints.get(&(next_t, false)) {
            Some(&edge) => edge.1 != next_loc,
            None => false,
        },
    }
}

/// Find the shortest path from start to goal such that it satisfies the given constraints.
pub fn astar(
    map: &[Vec<u8>],
    start_loc: &vertex::Vertex,
    goal_loc: &vertex::Vertex,
    h_values: &HashMap<vertex::Vertex, u16>,
    constraints: &[constraint::Constraint],
    min_path_length: usize,
) -> Option<Vec<vertex::Vertex>> {
    let mut open_list: BinaryHeap<Node> = BinaryHeap::new();
    let mut tree = Tree::new();

    let min_path_length = max(
        min_path_length as u16,
        constraints
            .iter()
            .filter(|c| !c.is_positive)
            .fold(0, |acc, c| acc.max(c.timestep)),
    );
    let neg_constraints: HashSet<(edge::Edge, bool, u16)> = constraints
        .iter()
        .filter(|c| !c.is_positive)
        .map(|c| (c.loc, c.is_edge, c.timestep))
        .collect();
    let pos_constraints: HashMap<(u16, bool), edge::Edge> = constraints
        .iter()
        .filter(|c| c.is_positive)
        .map(|c| ((c.timestep, c.is_edge), c.loc))
        .collect();

    let root_h_val = h_values[start_loc];
    open_list.push(tree.add_node(*start_loc, 0, root_h_val, 0, 0).unwrap());

    while !open_list.is_empty() {
        let cur_node = open_list.pop().unwrap();
        let cur_idx = tree.get_node_idx(cur_node);
        if cur_node.timestep >= min_path_length && cur_node.loc == *goal_loc {
            return Some(tree.get_path(cur_idx));
        }
        for action in 0..5 {
            let next_loc = match dijkstra::get_next_loc(map, cur_node.loc, action) {
                Some(vertex) => vertex,
                None => continue,
            };
            let next_t = cur_node.timestep + 1;
            if is_neg_constraint(cur_node.loc, next_loc, next_t, &neg_constraints)
                || is_pos_constraint(cur_node.loc, next_loc, next_t, &pos_constraints)
            {
                continue;
            }

            match tree.add_node(
                next_loc,
                cur_node.g_val + 1,
                h_values[&next_loc],
                next_t,
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
    duplicate_node: HashMap<(vertex::Vertex, u16), usize>,
}

impl Tree {
    fn new() -> Tree {
        Tree {
            tree: Vec::new(),
            parent_node: Vec::new(),
            duplicate_node: HashMap::new(),
        }
    }

    /// If node exists and is a shorter path, then update and return the node, otherwise none.
    /// If node does not exists, then add a new node and return the node
    fn add_node(
        &mut self,
        loc: vertex::Vertex,
        g_val: u16,
        h_val: u16,
        timestep: u16,
        parent: usize,
    ) -> Option<Node> {
        match self.duplicate_node.get(&(loc, timestep)) {
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
                self.tree.push(node);
                self.parent_node.push(parent);
                self.duplicate_node.insert((loc, timestep), node_idx);
                Some(node)
            }
        }
    }

    fn get_node_idx(&self, node: Node) -> usize {
        // *self.duplicate_node.get(&(node.loc, node.timestep)).unwrap()
        self.duplicate_node[&(node.loc, node.timestep)]
    }

    /// Runtime is O(c) where c is the path length.
    fn get_path(&self, goal_idx: usize) -> Vec<vertex::Vertex> {
        // Travel backwards from the goal node to the start node.
        let mut path: Vec<vertex::Vertex> = vec![self.tree[goal_idx].loc];
        let mut next_idx = self.parent_node[goal_idx];
        while next_idx != 0 {
            path.push(self.tree[next_idx].loc);
            next_idx = self.parent_node[next_idx];
        }
        path.push(self.tree[next_idx].loc); // Add start location
        path.shrink_to_fit();
        path.reverse();
        path
    }
}

use std::cmp::Ordering;

#[derive(Debug, Eq, Copy, Clone)]
struct Node {
    loc: vertex::Vertex,
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
    fn new(loc: vertex::Vertex, g_val: u16, h_val: u16, timestep: u16) -> Node {
        Node {
            loc,
            g_val,
            h_val,
            timestep,
        }
    }
}
