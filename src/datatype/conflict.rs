#[allow(clippy::enum_variant_names)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Conflict {
    Cardinal,
    SemiCardinal,
    NonCardinal,
}

impl Default for Conflict {
    fn default() -> Self {
        Conflict::NonCardinal
    }
}
