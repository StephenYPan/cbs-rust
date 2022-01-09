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

impl fmt::Debug for Location {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Location::Vertex(v) => f.debug_tuple("").field(&v.0).field(&v.1).finish(),
            Location::Edge(e) => f.debug_tuple("").field(&e.0).field(&e.1).finish(),
        }
    }
}

#[derive(Eq)]
pub struct Constraint {
    pub agent: u8,
    pub loc: Edge,
    pub is_edge: bool,
    pub timestep: u16,
    pub is_positive: bool,
}

impl PartialEq for Constraint {
    fn eq(&self, other: &Self) -> bool {
        self.loc.eq(&other.loc)
            && self.is_edge & other.is_edge
            && self.timestep == other.timestep
            && self.is_positive & other.is_positive
    }
}

impl Constraint {
    pub fn new(agent: u8, loc: Location, timestep: u16, is_positive: bool) -> Constraint {
        let location = match loc {
            Location::Vertex(v) => (Edge(Vertex(0, 0), v), false),
            Location::Edge(e) => (e, true),
        };
        Constraint {
            agent,
            loc: location.0,
            is_edge: location.1,
            timestep: timestep,
            is_positive: is_positive,
        }
    }
}

impl fmt::Debug for Constraint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let loc: Location = if self.is_edge {
            Location::new(self.loc)
        } else {
            Location::new(self.loc.1)
        };
        f.debug_struct("Constraint")
            .field("agent", &self.agent)
            .field("loc", &loc)
            .field("timestep", &self.timestep)
            .field("positive", &self.is_positive)
            .finish()
    }
}
