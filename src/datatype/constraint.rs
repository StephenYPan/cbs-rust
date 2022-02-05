use crate::datatype::{cardinal, edge, location, vertex};
use std::fmt;
use std::hash::{Hash, Hasher};

#[derive(Eq, Copy, Clone)]
pub struct Constraint {
    pub agent: u8,
    pub loc: edge::Edge,
    pub is_edge: bool,
    pub timestep: u16,
    pub is_positive: bool,
    pub cardinal: cardinal::Cardinal,
}

impl PartialEq for Constraint {
    fn eq(&self, other: &Self) -> bool {
        self.is_positive == other.is_positive
            && self.timestep == other.timestep
            && self.is_edge == other.is_edge
            && self.loc.eq(&other.loc)
    }
}

impl Hash for Constraint {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.loc.hash(state);
        self.is_edge.hash(state);
        self.timestep.hash(state);
        self.is_positive.hash(state);
    }
}

impl Constraint {
    pub fn new(
        agent: u8,
        loc: location::Location,
        timestep: u16,
        is_positive: bool,
        cardinal: cardinal::Cardinal,
    ) -> Constraint {
        let location = match loc {
            location::Location::Vertex(v) => (edge::Edge(vertex::Vertex(0, 0), v), false),
            location::Location::Edge(e) => (e, true),
        };
        Constraint {
            agent,
            loc: location.0,
            is_edge: location.1,
            timestep,
            is_positive,
            cardinal,
        }
    }
}

impl fmt::Debug for Constraint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let loc: location::Location = if self.is_edge {
            location::Location::new(self.loc)
        } else {
            location::Location::new(self.loc.1)
        };
        f.debug_struct("Constraint")
            .field("agent", &self.agent)
            .field("loc", &loc)
            .field("timestep", &self.timestep)
            .field("positive", &self.is_positive)
            .finish()
    }
}
