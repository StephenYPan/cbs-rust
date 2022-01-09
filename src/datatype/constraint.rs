use crate::datatype::edge::Edge;

#[derive(Debug)]
pub struct Constraint {
    pub loc: Edge,
    pub is_edge: bool,
    pub timestep: u16,
    pub is_positive: bool,
}

impl Constraint {
    pub fn new(loc: Edge, is_edge: bool, timestep: u16, is_positive: bool) -> Constraint {
        Constraint {
            loc: loc,
            is_edge: is_edge,
            timestep: timestep,
            is_positive: is_positive,
        }
    }
}
