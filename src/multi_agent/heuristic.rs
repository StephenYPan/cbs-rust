use crate::datatype::{cardinal, collision, constraint};
use crate::map_reader::map;
use crate::multi_agent::{cbs, mdd};
use std::cmp::{max, min};
use std::sync::{Arc, Mutex};
use std::thread;

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Heuristic {
    Cg,
    Dg,
    Wdg,
    None,
}

/// Get next bit permutation of greater value from the given set.
fn gospers_hack(set: i32) -> i32 {
    // Gosper's hack
    // Cycle through all permutations of bit ordering in increasing
    // order to find an ordering that is a vertex cover.
    let c = set & -set;
    let r = set + c;
    (((r ^ set) >> 2) / c) | r
}

fn is_vertex_cover(
    adj_matrix: &[Vec<u8>],
    num_vertices: usize,
    num_edges: usize,
    k: usize,
) -> bool {
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
                    if (visited[i][j] > 0) || adj_matrix[i][j] == 0 {
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

fn find_min_vertex_cover(adj_matrix: &[Vec<u8>], num_vertices: usize, num_edges: usize) -> u16 {
    if num_edges <= 1 {
        return num_edges as u16;
    }
    let mut low = 1; // Lower bound for minimum vertex cover
    let mut high = min(num_edges + 1, num_vertices);
    while low < high {
        let mid = (low + high) >> 1;
        if is_vertex_cover(adj_matrix, num_vertices, num_edges, mid) {
            high = mid;
        } else {
            low = mid + 1;
        }
    }
    low as u16
}

/// Calculate and generate a graph based only on full cardinal conflicts.
pub fn cg_heuristic(collisions: &[collision::Collision], num_agents: usize) -> u16 {
    let num_vertices = num_agents;
    let mut num_edges: usize = 0;
    let mut adj_matrix: Vec<Vec<u8>> = vec![vec![0; num_vertices]; num_vertices];
    for collision in collisions {
        if collision.cardinal != cardinal::Cardinal::Full {
            continue;
        }
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;
        adj_matrix[a1][a2] = 1;
        adj_matrix[a2][a1] = 1;
        num_edges += 1;
    }
    find_min_vertex_cover(&adj_matrix, num_vertices, num_edges)
}

/// Calculate and generate a graph based only on full and semi cardinal conflicts.
pub fn dg_heuristic(collisions: &[collision::Collision], num_agents: usize) -> u16 {
    let num_vertices = num_agents;
    let mut num_edges: usize = 0;
    let mut adj_matrix: Vec<Vec<u8>> = vec![vec![0; num_vertices]; num_vertices];
    for collision in collisions {
        if collision.cardinal == cardinal::Cardinal::Non {
            continue;
        }
        let a1 = collision.a1 as usize;
        let a2 = collision.a2 as usize;
        adj_matrix[a1][a2] = 1;
        adj_matrix[a2][a1] = 1;
        num_edges += 1;
    }
    find_min_vertex_cover(&adj_matrix, num_vertices, num_edges)
}

/// Get disjoint subgraphs of a given graph.
fn get_subgraph(adj_matrix: &[Vec<u8>]) -> Vec<Vec<u8>> {
    let mut subgraph: Vec<Vec<u8>> = Vec::new();
    let mut open_list: Vec<u8> = Vec::new();
    let mut closed_list: Vec<bool> = vec![false; adj_matrix.len()];
    for (i, row) in adj_matrix.iter().enumerate() {
        if closed_list[i] {
            continue;
        }
        closed_list[i] = true;
        let mut group: Vec<u8> = vec![i as u8];
        for (j, v) in row.iter().enumerate() {
            // Only add dependent vertices
            if *v > 0 {
                open_list.push(j as u8);
            }
        }
        while !open_list.is_empty() {
            let next_v = open_list.pop().unwrap();
            if closed_list[next_v as usize] {
                continue;
            }
            closed_list[next_v as usize] = true;
            group.push(next_v);
            for (k, v) in adj_matrix[next_v as usize].iter().enumerate() {
                // Only add dependent vertices
                if *v > 0 {
                    open_list.push(k as u8);
                }
            }
        }
        subgraph.push(group);
    }
    subgraph
}

fn find_min_weight_vertex_cover(adj_matrix: Vec<Vec<u8>>) -> u16 {
    // Allow each subgraph to calculate its own min edge weight mvc, and sum up the result.
    let graph = get_subgraph(&adj_matrix);
    let result = Arc::new(Mutex::new(0));
    let mut handles = vec![];

    for subgraph in graph {
        match subgraph.len() {
            2 => {
                let edge_weight = adj_matrix[subgraph[0] as usize][subgraph[1] as usize] as u16;
                let mut res = result.lock().unwrap();
                *res += edge_weight;
            }
            n if n > 2 => {
                let subgraph = subgraph.clone();
                let result = Arc::clone(&result);
                let adj_matrix = adj_matrix.clone();
                let handle = thread::spawn(move || {
                    let mut sub_num_edges = 0;
                    let mut sub_adj_matrix: Vec<Vec<u8>> = vec![vec![0; n]; n];
                    for (i, agent1) in subgraph[0..n - 1].iter().enumerate() {
                        for (j, agent2) in subgraph[i..].iter().enumerate() {
                            let edge_weight = adj_matrix[*agent1 as usize][*agent2 as usize];
                            sub_adj_matrix[i][j + i] = edge_weight;
                            sub_adj_matrix[j + i][i] = edge_weight;
                            if edge_weight > 0 {
                                sub_num_edges += 1;
                            }
                        }
                    }
                    let mvc = find_min_vertex_cover(&sub_adj_matrix, n, sub_num_edges);
                    if mvc == 1 {
                        let max =
                            *sub_adj_matrix.iter().flat_map(|v| v.iter()).max().unwrap() as u16;
                        let mut res = result.lock().unwrap();
                        *res += max;
                    } else {
                        // Find mwvc through exhaustive search
                        let mut min_weight = u16::MAX;
                        let mut set = (1 << mvc) - 1;
                        let limit = 1 << n;
                        while set < limit {
                            let mut visited: Vec<Vec<u8>> = vec![vec![0; n]; n];
                            let mut num_edges_visited = 0;
                            let mut k = 1;
                            let mut i = 0;
                            while k < limit {
                                if (set & k) > 0 {
                                    for j in 0..n {
                                        if (visited[i][j] > 0) || sub_adj_matrix[i][j] == 0 {
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
                            if num_edges_visited == sub_num_edges {
                                let mut min_cover_agents: Vec<usize> = Vec::with_capacity(n);
                                for (i, c) in format!("{:b}", set).chars().rev().enumerate() {
                                    // Get the corresponding agents that are the mvc
                                    if c == '1' {
                                        min_cover_agents.push(i);
                                    }
                                }
                                // Find the min weight vertex cover
                                let mut vertex_weights: Vec<u8> = vec![0; n];
                                for v in &min_cover_agents {
                                    for u in 0..n {
                                        if min_cover_agents.contains(&u) {
                                            continue;
                                        }
                                        let edge_weight = sub_adj_matrix[*v][u];
                                        vertex_weights[*v] = edge_weight;
                                    }
                                }
                                for v in &min_cover_agents {
                                    let v_weight = vertex_weights[*v];
                                    for u in &min_cover_agents {
                                        let edge_weight = sub_adj_matrix[*v][*u];
                                        let u_weight = vertex_weights[*u];
                                        if v_weight + u_weight >= edge_weight {
                                            continue;
                                        }
                                        vertex_weights[*v] += edge_weight - (v_weight + u_weight);
                                    }
                                }
                                min_weight =
                                    min(min_weight, vertex_weights.iter().sum::<u8>() as u16);
                            }
                            set = gospers_hack(set);
                        }
                        let mut res = result.lock().unwrap();
                        *res += min_weight;
                    }
                });
                handles.push(handle);
            }
            _ => {}
        }
    }
    for handle in handles {
        handle.join().unwrap();
    }
    let result = *result.lock().unwrap();
    result
}

/// Generate a graph based only on full and semi cardinal conflicts, then run cbs
/// on the conflicting agents to generate the edge cost. If no solution is found
/// between the two conflicting agents, then the edge cost is defaulted to 1.
pub fn wdg_heuristic(
    map_instance: &map::MapInstance,
    constraints: &[constraint::Constraint],
    collisions: &[collision::Collision],
    mdds: Vec<mdd::Mdd>,
) -> u16 {
    let num_vertices = mdds.len();
    let adj_matrix = Arc::new(Mutex::new(vec![vec![0; num_vertices]; num_vertices]));
    let mut handles = vec![];

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
        let adj_matrix = Arc::clone(&adj_matrix);
        let mdds = mdds.clone();
        let handle = thread::spawn(move || {
            let heuristics = vec![false, true, false];
            let paths = cbs::cbs(
                &sub_map_instance,
                Some(sub_constraints),
                true, // child process
                true, // disjoint
                heuristics,
            );
            let mut adj_matrix = adj_matrix.lock().unwrap();
            match paths {
                Some(paths) => {
                    // mdd length goes from 0 to path.len() - 2, to match
                    // it with the new path we take the new_path.len() - 1.
                    let path_len1 = mdds[a1].mdd.len();
                    let path_len2 = mdds[a2].mdd.len();
                    let new_path_len1 = paths[0].len() - 1;
                    let new_path_len2 = paths[1].len() - 1;
                    let edge_weight = max(
                        new_path_len1.saturating_sub(path_len1),
                        new_path_len2.saturating_sub(path_len2),
                    );
                    // NOTE: Is this optimal? If the path cost stayed the same, should
                    // we exclude it from the heuristics?
                    // edge_weight = max(edge_weight, 1);
                    adj_matrix[a1][a2] = edge_weight as u8;
                    adj_matrix[a2][a1] = edge_weight as u8;
                }
                None => {
                    adj_matrix[a1][a2] = 1;
                    adj_matrix[a2][a1] = 1;
                }
            }
        });
        handles.push(handle);
    }
    for handle in handles {
        handle.join().unwrap();
    }
    let res = find_min_weight_vertex_cover(adj_matrix.lock().unwrap().to_vec());
    res
}
