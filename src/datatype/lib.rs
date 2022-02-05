use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

pub fn hash<T>(vec: &[T]) -> u64
where
    T: Hash,
{
    let mut state = DefaultHasher::new();
    vec.hash(&mut state);
    state.finish()
}
