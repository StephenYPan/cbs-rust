use crate::datatype::{constraint::Location, edge::Edge, vertex::Vertex};
use std::fmt;

#[derive(Eq)]
pub struct Collision {
    pub a1: u8,
    pub a2: u8,
    pub loc: Edge,
    pub is_edge: bool,
    pub timestep: u16,
}

impl PartialEq for Collision {
    fn eq(&self, other: &Self) -> bool {
        self.loc.eq(&other.loc) && self.is_edge & other.is_edge && self.timestep == other.timestep
    }
}

impl Collision {
    pub fn new(a1: u8, a2: u8, loc: Location, timestep: u16) -> Collision {
        let location = match loc {
            Location::Vertex(v) => (Edge(Vertex(0, 0), v), false),
            Location::Edge(e) => (e, true),
        };
        Collision {
            a1: a1,
            a2: a2,
            loc: location.0,
            is_edge: location.1,
            timestep: timestep,
        }
    }
}

impl fmt::Debug for Collision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let loc: Location = if self.is_edge {
            Location::new(self.loc)
        } else {
            Location::new(self.loc.1)
        };
        f.debug_struct("Collision")
            .field("a1", &self.a1)
            .field("a2", &self.a2)
            .field("loc", &loc)
            .field("timestep", &self.timestep)
            .finish()
    }
}
