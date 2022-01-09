use crate::datatype::{edge::Edge, vertex::Vertex};
use std::fmt;

pub enum Location {
    Vertex(Vertex),
    Edge(Edge),
}

pub trait IntoLocation {
    fn into(self) -> Location;
}

impl IntoLocation for Vertex {
    fn into(self) -> Location {
        Location::Vertex(self)
    }
}

impl IntoLocation for Edge {
    fn into(self) -> Location {
        Location::Edge(self)
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

pub struct Constraints {
    constraints: Vec<(u8, Edge, bool, u16, bool)>,
}

impl Constraints {
    pub fn new() -> Constraints {
        Constraints {
            constraints: Vec::new(),
        }
    }

    pub fn push(&mut self, loc: Location, timestep: u16, agent: u8, is_positive: bool) {
        match loc {
            Location::Vertex(v) => {
                let e = Edge(Vertex(0, 0), v);
                self.constraints
                    .push((agent, e, false, timestep, is_positive))
            }
            Location::Edge(e) => self
                .constraints
                .push((agent, e, true, timestep, is_positive)),
        }
    }
}

impl fmt::Debug for Constraints {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.constraints.iter()).finish()
    }
}
