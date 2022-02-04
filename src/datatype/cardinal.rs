#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Cardinal {
    Full,
    Semi,
    Non,
}

impl Default for Cardinal {
    fn default() -> Self {
        Cardinal::Non
    }
}
