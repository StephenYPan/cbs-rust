use crate::datatype::{collision, mdd};
use std::collections::{hash_map::DefaultHasher, HashMap, HashSet};
use std::hash::{Hash, Hasher};

pub fn hash_mdds(collisions: &[collision::Collision], mdds: &[mdd::Mdd]) -> HashMap<usize, u64> {
    let mut mdd_hashes: HashMap<usize, u64> = HashMap::new();
    let agents: HashSet<usize> = collisions
        .iter()
        .flat_map(|c| vec![c.a1 as usize, c.a2 as usize])
        .collect();
    for i in agents.iter() {
        let mut state = DefaultHasher::new();
        for layer in &mdds[*i].mdd {
            layer.hash(&mut state);
        }
        let hash = state.finish();
        mdd_hashes.insert(*i, hash);
    }
    mdd_hashes.shrink_to_fit();
    mdd_hashes
}

/// Mutates the input vector of collisions by replacing the collisions
/// with cardinal or semi-cardinal collisions if applicable.
pub fn detect_cardinal_conflicts(collisions: &mut [collision::Collision], mdds: &[mdd::Mdd]) {
    let mut conflict_index: Vec<usize> = Vec::new();
    let mut conflicts: Vec<collision::Collision> = Vec::new();
    let mdd_hashes: HashMap<usize, u64> = hash_mdds(collisions, mdds);

    for (i, collision) in collisions.iter().enumerate() {
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;
        let joint_mdd_hash = mdd_hashes[&a1] ^ mdd_hashes[&a2];

        if let Some(cardinal_conflict) =
            mdd::find_cardinal_conflict(&mdds[a1], &mdds[a2], a1 as u8, a2 as u8, joint_mdd_hash)
        {
            conflict_index.push(i);
            conflicts.push(cardinal_conflict);
            continue;
        }
        if let Some(semi_cardinal_conflict) =
            mdd::find_dependency_conflict(&mdds[a1], &mdds[a2], a1 as u8, a2 as u8, joint_mdd_hash)
        {
            conflict_index.push(i);
            conflicts.push(semi_cardinal_conflict);
            continue;
        }
    }
    for (i, c) in conflict_index.iter().zip(conflicts) {
        collisions[*i] = c;
    }
}
