use crate::datatype::vertex;

pub struct MapInstance {
    pub map: Vec<Vec<u8>>,
    pub starts: Vec<vertex::Vertex>,
    pub goals: Vec<vertex::Vertex>,
}
