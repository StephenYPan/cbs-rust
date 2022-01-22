use crate::datatype::vertex::Vertex;

pub struct MapInstance {
    pub map: Vec<Vec<u8>>,
    pub starts: Vec<Vertex>,
    pub goals: Vec<Vertex>,
}
