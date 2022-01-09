use crate::datatype::constraint::Location;
use std::fmt;

#[derive(Eq)]
pub struct Collision {
    pub a1: u8,
    pub a2: u8,
    pub loc: Location,
    pub timestep: u16,
}

impl PartialEq for Collision {
    fn eq(&self, other: &Self) -> bool {
        self.timestep == other.timestep && self.loc.eq(&other.loc)
    }
}

impl Collision {
    pub fn new(a1: u8, a2: u8, loc: Location, timestep: u16) -> Collision {
        Collision {
            a1: a1,
            a2: a2,
            loc: loc,
            timestep: timestep,
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
            .finish()
    }
}
