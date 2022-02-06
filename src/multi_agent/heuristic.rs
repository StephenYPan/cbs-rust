use crate::datatype::{cardinal, collision, constraint};
use crate::map_reader::map;
use crate::multi_agent::{cbs, mdd};
use std::cmp::{max, min};

/// Get next bit permutation of greater value from the given set.
fn gospers_hack(set: i32) -> i32 {
    // Gosper's hack
    // Cycle through all permutations of bit ordering in increasing
    // order to find an ordering that is a vertex cover.
    let c = set & -set;
    let r = set + c;
    (((r ^ set) >> 2) / c) | r
}

fn is_vertex_cover(graph: &[Vec<u8>], num_vertices: usize, num_edges: usize, k: usize) -> bool {
    let mut set = (1 << k) - 1;
    let limit = 1 << num_vertices;
    while set < limit {
        let mut visited: Vec<Vec<u8>> = vec![vec![0; num_vertices]; num_vertices];
        let mut num_edges_visited = 0;
        let mut k = 1;
        let mut i = 0;
        while k < limit {
            if (set & k) > 0 {
                for j in 0..num_vertices {
                    if visited[i][j] != graph[i][j] {
                        continue;
                    }
                    visited[i][j] = 1;
                    visited[j][i] = 1;
                    num_edges_visited += 1;
                }
            }
            k <<= 1;
            i += 1;
        }
        if num_edges_visited == num_edges {
            return true;
        }
        set = gospers_hack(set);
    }
    false
}

fn find_min_vertex_cover(graph: Vec<Vec<u8>>, num_vertices: usize, num_edges: usize) -> u16 {
    if num_edges <= 1 {
        return num_edges as u16;
    }
    let mut low = 1; // Lower bound for minimum vertex cover
    let mut high = min(num_edges + 1, num_vertices);
    while low < high {
        let mid = (low + high) >> 1;
        if is_vertex_cover(&graph, num_vertices, num_edges, mid) {
            high = mid;
        } else {
            low = mid + 1;
        }
    }
    low as u16
}

/// Calculate and generate a graph based only on full cardinal conflicts.
pub fn cg_heuristic(collisions: &[collision::Collision], mdds: &[mdd::Mdd]) -> u16 {
    let num_vertices = mdds.len();
    let mut num_edges: usize = 0;
    let mut graph: Vec<Vec<u8>> = vec![vec![0; num_vertices]; num_vertices];
    for collision in collisions {
        if collision.cardinal != cardinal::Cardinal::Full {
            continue;
        }
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;
        graph[a1][a2] = 1;
        graph[a2][a1] = 1;
        num_edges += 1;
    }
    find_min_vertex_cover(graph, num_vertices, num_edges)
}

/// Calculate and generate a graph based only on full and semi cardinal conflicts.
pub fn dg_heuristic(collisions: &[collision::Collision], mdds: &[mdd::Mdd]) -> u16 {
    let num_vertices = mdds.len();
    let mut num_edges: usize = 0;
    let mut graph: Vec<Vec<u8>> = vec![vec![0; num_vertices]; num_vertices];
    for collision in collisions {
        if collision.cardinal == cardinal::Cardinal::Non {
            continue;
        }
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;
        graph[a1][a2] = 1;
        graph[a2][a1] = 1;
        num_edges += 1;
    }
    find_min_vertex_cover(graph, num_vertices, num_edges)
}

#[allow(unused)]
fn find_min_edge_weight_min_vertex_cover(
    graph: Vec<Vec<u8>>,
    num_vertices: usize,
    num_edges: usize,
) -> u16 {
    // Gosper's hack
    // let mvc_set = find_min_vertex_cover(graph, num_vertices, num_edges);
    0
}

#[allow(unused)]
/// Generate a graph based only on full and semi cardinal conflicts, then run cbs
/// on the conflicting agents to generate the edge cost. If no solution is found
/// between the two conflicting agents, then the edge cost is defaulted to 1.
pub fn wdg_heuristic(
    map_instance: &map::MapInstance,
    constraints: &[constraint::Constraint],
    collisions: &[collision::Collision],
    mdds: &[mdd::Mdd],
) -> u16 {
    let num_vertices = mdds.len();
    // num_edges and graph are shared variables, should be atomic
    let mut num_edges: usize = 0;
    let mut graph: Vec<Vec<u8>> = vec![vec![0; num_vertices]; num_vertices];

    // TODO: Parallelize this search
    for collision in collisions {
        if collision.cardinal == cardinal::Cardinal::Non {
            continue;
        }
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;

        let sub_map_instance: map::MapInstance = map::MapInstance {
            map: map_instance.map.clone(),
            starts: vec![map_instance.starts[a1], map_instance.starts[a2]],
            goals: vec![map_instance.goals[a1], map_instance.goals[a2]],
        };
        let mut sub_constraints: Vec<constraint::Constraint> = constraints
            .iter()
            .filter(|c| c.agent == collision.a1 || c.agent == collision.a2)
            .copied()
            .collect();
        for c in &mut sub_constraints {
            c.agent = if c.agent == a1 as u8 { 0 } else { 1 };
        }
        let heuristics = vec![false, true, false];
        let paths = cbs::cbs(
            &sub_map_instance,
            Some(sub_constraints),
            true,
            true,
            heuristics,
        );
        match paths {
            Some(paths) => {
                // mdd length goes from 0 to path.len() - 2, to match
                // it with the new path we take the new_path.len() - 1.
                let path_len1 = mdds[a1].mdd.len();
                let path_len2 = mdds[a2].mdd.len();
                let new_path_len1 = paths[0].len() - 1;
                let new_path_len2 = paths[1].len() - 1;
                // TODO: Is this max or min?
                let mut edge_weight = max(
                    new_path_len1.saturating_sub(path_len1),
                    new_path_len2.saturating_sub(path_len2),
                );
                edge_weight = max(edge_weight, 1);
                graph[a1][a2] = edge_weight as u8;
                graph[a2][a1] = edge_weight as u8;
            }
            None => {
                graph[a1][a2] = 1;
                graph[a2][a1] = 1;
            }
        }
        num_edges += 1;
    }
    find_min_edge_weight_min_vertex_cover(graph, num_vertices, num_edges)
}
