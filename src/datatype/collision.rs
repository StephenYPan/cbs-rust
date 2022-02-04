use crate::datatype::{conflict, constraint};
use std::fmt;

#[derive(Eq, Copy, Clone)]
pub struct Collision {
    pub a1: u8,
    pub a2: u8,
    pub loc: constraint::Location,
    pub timestep: u16,
    pub conflict: conflict::Conflict,
}

impl PartialEq for Collision {
    fn eq(&self, other: &Self) -> bool {
        self.timestep == other.timestep && self.loc.eq(&other.loc)
    }
}

impl Collision {
    pub fn new(
        a1: u8,
        a2: u8,
        loc: constraint::Location,
        timestep: u16,
        conflict: conflict::Conflict,
    ) -> Collision {
        Collision {
            a1,
            a2,
            loc,
            timestep,
            conflict,
        }
    }
}

impl fmt::Debug for Collision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Collision")
            .field("a1", &self.a1)
            .field("a2", &self.a2)
            .field("loc", &self.loc)
            .field("timestep", &self.timestep)
            .field("conflict", &self.conflict)
            .finish()
    }
}
