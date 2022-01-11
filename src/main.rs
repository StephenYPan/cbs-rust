mod datatype;
mod multi_agent;
mod single_agent;

use datatype::vertex::Vertex;
use multi_agent::cbs::cbs;
#[allow(unused)]
use std::mem::size_of_val;
use std::time::Instant;

fn main() {
    // // 4 4
    // // . . . @
    // // . . . @
    // // . . . .
    // // @ @ . .
    // // 2
    // // 1 0 2 3
    // // 0 1 3 2
    // let mut map: Vec<Vec<u8>> = vec![vec![1; 4]; 4];
    // map[0][3] = 0;
    // map[1][3] = 0;
    // map[3][0] = 0;
    // map[3][1] = 0;
    // let starts: Vec<Vertex> = vec![Vertex(1, 0), Vertex(0, 1)];
    // let goals: Vec<Vertex> = vec![Vertex(2, 3), Vertex(3, 2)];

    // 4 5
    // . . @ @ .
    // . . . . .
    // . . @ @ .
    // . . @ @ .
    // 3
    // 2 1 2 4
    // 0 4 0 1
    // 1 0 3 4
    let mut map: Vec<Vec<u8>> = vec![vec![1; 5]; 4];
    map[0][2] = 0;
    map[0][3] = 0;
    map[2][2] = 0;
    map[2][3] = 0;
    map[3][2] = 0;
    map[3][3] = 0;
    let starts: Vec<Vertex> = vec![Vertex(2, 1), Vertex(0, 4), Vertex(1, 0)];
    let goals: Vec<Vertex> = vec![Vertex(2, 4), Vertex(0, 1), Vertex(3, 4)];

    // // . @ .
    // // . . .
    // // . @ .
    // // 5 + 6
    // let mut map: Vec<Vec<u8>> = vec![vec![1; 3]; 3];
    // map[0][1] = 0;
    // map[2][1] = 0;
    // let starts: Vec<Vertex> = vec![Vertex(1, 0), Vertex(1, 2)];
    // let goals: Vec<Vertex> = vec![Vertex(1, 2), Vertex(1, 0)];

    for r in &map {
        println!("{:?}", r);
    }
    println!("{:?}", starts);
    println!("{:?}", goals);
    println!();

    let disjoint = true;

    let now = Instant::now();
    let paths = cbs(&map, starts, goals, None, disjoint);
    let elapsed_time = now.elapsed();
    println!("Search time: {:?}", elapsed_time);
    println!();

    match paths {
        Some(paths) => {
            for (i, p) in paths.iter().enumerate() {
                println!("agent-{}, path: {:?}", i, p);
            }
        }
        None => println!("No solution."),
    }
}
