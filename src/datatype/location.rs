use crate::datatype::{edge, vertex};
use std::fmt;

#[derive(PartialEq, Eq, Copy, Clone)]
pub enum Location {
    Vertex(vertex::Vertex),
    Edge(edge::Edge),
}

pub trait IntoLocation {
    fn into(self) -> Location;
}

impl IntoLocation for vertex::Vertex {
    fn into(self) -> Location {
        Location::Vertex(self)
    }
}

impl IntoLocation for edge::Edge {
    fn into(self) -> Location {
        Location::Edge(self)
    }
}

impl Default for Location {
    fn default() -> Self {
        Location::Vertex(vertex::Vertex(0, 0))
    }
}

impl Location {
    pub fn new<L>(loc: L) -> Location
    where
        L: IntoLocation,
    {
        loc.into()
    }
}

impl fmt::Debug for Location {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Location::Vertex(v) => f.debug_tuple("").field(&v.0).field(&v.1).finish(),
            Location::Edge(e) => f.debug_tuple("").field(&e.0).field(&e.1).finish(),
        }
    }
}
