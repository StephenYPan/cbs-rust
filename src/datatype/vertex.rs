use std::fmt;

#[derive(Eq, PartialEq, Hash, Copy, Clone)]
pub struct Vertex(pub u16, pub u16);

impl fmt::Debug for Vertex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("").field(&self.0).field(&self.1).finish()
    }
}
