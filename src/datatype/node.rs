use crate::datatype::vertex::Vertex;
use std::cmp::Ordering;

#[derive(Debug, Eq, Copy, Clone)]
pub struct Node {
    pub loc: Vertex,
    pub g_val: u16,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.g_val.eq(&other.g_val)
    }
}

impl PartialEq<Vertex> for Node {
    fn eq(&self, other: &Vertex) -> bool {
        self.loc == *other
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.g_val.cmp(&other.g_val).reverse()
    }
}

impl Node {
    pub fn new(loc: Vertex, g_val: u16) -> Node {
        Node {
            loc: loc,
            g_val: g_val,
        }
    }
}
