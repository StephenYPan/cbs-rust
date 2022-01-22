use crate::datatype::vertex;
use std::fmt;

#[derive(Eq, PartialEq, Hash, Copy, Clone)]
pub struct Edge(pub vertex::Vertex, pub vertex::Vertex);

impl fmt::Debug for Edge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("").field(&self.0).field(&self.1).finish()
    }
}
