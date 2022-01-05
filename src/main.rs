// #[path = "./pathfinding/dijkstra.rs"]
// mod dijkstra;
mod datatype;
mod pathfinding;

use datatype::vertex;
use std::mem::size_of_val;

fn main() {
    // 4 4
    // . . . @
    // . . . @
    // . . . .
    // @ @ . .
    // 2
    // 1 0 2 3
    // 0 1 3 2
    let mut map: Vec<Vec<u8>> = vec![vec![1; 4]; 4];
    map[0][3] = 0;
    map[1][3] = 0;
    map[3][0] = 0;
    map[3][1] = 0;
    for r in &map {
        println!("{:?}", r);
    }
    println!("map size (bytes): {:?}", size_of_val(&map));
    let start_loc = vertex::Vertex(1, 0);
    println!("vertex size (bytes): {:?}", size_of_val(&start_loc));
    let h_values = pathfinding::dijkstra::compute_heuristics(&map, start_loc);
    // println!("{:?}", start_loc);
    for (vertex, g_val) in &h_values {
        println!("{:?} {:?}", vertex, g_val);
    }
    println!("h_values size (bytes): {:?}", size_of_val(&h_values));
    // println!("h_values len: {:?}", h_values.len());
    // let goal_loc = vertex::Vertex(2, 3);
}
