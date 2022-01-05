use crate::datatype::vertex::Vertex;

#[derive(Debug, Eq, PartialEq, Hash, Copy, Clone)]
pub struct Edge(pub Vertex, pub Vertex);
