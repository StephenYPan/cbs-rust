mod datatype;
mod multi_agent;
mod single_agent;

use datatype::vertex::Vertex;
use multi_agent::cbs::cbs;
#[allow(unused)]
use std::mem::size_of_val;
#[allow(unused)]
use std::time::Instant;

fn main() {
    // // . . . @
    // // . . . @
    // // . . . .
    // // @ @ . .
    // let mut map: Vec<Vec<u8>> = vec![vec![1; 4]; 4];
    // map[0][3] = 0;
    // map[1][3] = 0;
    // map[3][0] = 0;
    // map[3][1] = 0;
    // let starts: Vec<Vertex> = vec![Vertex(1, 0), Vertex(0, 1)];
    // let goals: Vec<Vertex> = vec![Vertex(2, 3), Vertex(3, 2)];

    // . . @ @ .
    // . . . . .
    // . . @ @ .
    // . . @ @ .
    // let mut map: Vec<Vec<u8>> = vec![vec![1; 5]; 4];
    // map[0][2] = 0;
    // map[0][3] = 0;
    // map[2][2] = 0;
    // map[2][3] = 0;
    // map[3][2] = 0;
    // map[3][3] = 0;
    // let starts: Vec<Vertex> = vec![Vertex(2, 1), Vertex(0, 4), Vertex(1, 0)];
    // let goals: Vec<Vertex> = vec![Vertex(2, 4), Vertex(0, 1), Vertex(3, 4)];

    // . . . . . . . @
    // @ . @ . . . . .
    // @ @ . . . @ . .
    // . @ . . . . @ .
    // . . @ . . . . .
    // . . . . . . . .
    // . @ @ @ . . . .
    // . . . . . . . .
    let mut map: Vec<Vec<u8>> = vec![vec![1; 8]; 8];
    map[1][0] = 0;
    map[2][0] = 0;
    map[2][1] = 0;
    map[3][1] = 0;
    map[6][1] = 0;
    map[1][2] = 0;
    map[4][2] = 0;
    map[6][2] = 0;
    map[6][3] = 0;
    map[2][5] = 0;
    map[3][6] = 0;
    map[0][7] = 0;
    let starts: Vec<Vertex> = vec![
        Vertex(1, 5),
        Vertex(4, 0),
        Vertex(0, 4),
        Vertex(3, 0),
        Vertex(6, 4),
        Vertex(0, 3),
        Vertex(6, 0),
    ];
    let goals: Vec<Vertex> = vec![
        Vertex(7, 0),
        Vertex(2, 2),
        Vertex(4, 3),
        Vertex(7, 6),
        Vertex(3, 4),
        Vertex(4, 0),
        Vertex(0, 5),
    ];

    // for r in &map {
    //     println!("{:?}", r);
    // }
    // println!("{:?}", starts);
    // println!("{:?}", goals);
    // println!();

    let disjoint = true;

    // let now = Instant::now();
    let paths = cbs(&map, starts, goals, None, disjoint);
    // let elapsed_time = now.elapsed();
    // println!("Search time: {:?}", elapsed_time);
    // println!();
    #[allow(unused)]
    match paths {
        Some(paths) => {
            // for (i, p) in paths.iter().enumerate() {
            //     println!("agent-{}, path: {:?}", i, p);
            // }
        }
        None => println!("No solution."),
    }
}
