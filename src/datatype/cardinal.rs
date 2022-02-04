#[allow(clippy::enum_variant_names)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Cardinal {
    Full,
    Semi,
    None,
}

impl Default for Cardinal {
    fn default() -> Self {
        Cardinal::None
    }
}
