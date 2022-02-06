use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

/// Ordered hash a vector. Returns a hash value.
pub fn hash<T>(vec: &[T]) -> u64
where
    T: Hash,
{
    let mut state = DefaultHasher::new();
    vec.hash(&mut state);
    state.finish()
}

/// Ordered hash of a matrix. Returns a hash value.
pub fn hash2d<T>(matrix: &[Vec<T>]) -> u64
where
    T: Hash,
{
    let mut state = DefaultHasher::new();
    for row in matrix.iter() {
        row.hash(&mut state);
    }
    state.finish()
}
