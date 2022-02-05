use crate::datatype::{cardinal, collision, mdd};
use std::cmp::min;

fn is_vertex_cover(graph: &[Vec<u8>], num_vertices: usize, num_edges: usize, k: usize) -> bool {
    let mut set = (1 << k) - 1;
    let limit = 1 << num_vertices;
    while set < limit {
        let mut visited: Vec<Vec<u8>> = vec![vec![0; num_vertices]; num_vertices];
        let mut num_edges_visited = 0;
        let mut k = 1;
        let mut i = 0;
        while k < limit {
            if (set & k) == 1 {
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
        // Gosper's hack
        // Cycle through all permutations of vertices to find a valid vertex cover.
        let c = set & -set;
        let r = set + c;
        set = (((r ^ set) >> 2) / c) | r;
    }
    false
}

fn calc_min_vertex_cover(graph: Vec<Vec<u8>>, num_vertices: usize, num_edges: usize) -> u16 {
    let mut low = 1; // Minimum number of vertices as a vertex cover if there are at least 1 edge
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
    if num_edges <= 1 {
        return num_edges as u16;
    }
    calc_min_vertex_cover(graph, num_vertices, num_edges)
}

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
    if num_edges <= 1 {
        return num_edges as u16;
    }
    calc_min_vertex_cover(graph, num_vertices, num_edges)
}

// pub fn wdg_heuristic(collisions: &[collision::Collision], mdds: &[mdd::Mdd]) {}
