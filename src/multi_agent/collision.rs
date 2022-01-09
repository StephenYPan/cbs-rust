use crate::datatype::{constraint::Location, edge::Edge};
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
