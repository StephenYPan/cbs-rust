use std::fs;
use std::io::{self, BufRead};
// use std::io::self;

fn main() {
    import_mapf_instance("instances/exp0.txt");
    println!("Hello, world!");
}

fn import_mapf_instance(filename: &str) -> Vec<Vec<u8>> {
    // let text = fs::read_to_string(filename).expect("Error on import_mapf_instance()");
    // println!("{}", text);
    let file = fs::File::open(filename).expect("ERROR");
    let mut text = io::BufReader::new(file);
    let mut line = String::new();
    text.read_line(&mut line).expect("ERROR");
    let vec: Vec<&str> = line.split_whitespace().collect();
    // println!("{}", vec[0]);
    println!("{}", vec[1]);
    let row = vec[0].parse::<usize>().unwrap();
    // println!("{}", row);
    let col = vec[1].parse::<usize>().unwrap();
    // println!("{}", col);
    println!("{}", line);
    let mut map: Vec<Vec<u8>> = vec![vec![1; col]; row];
    for i in 0..row {
        line.clear();
        text.read_line(&mut line).expect("ERROR");
        let rows: Vec<&str> = line.split_whitespace().collect();
        for (j, r) in rows.iter().enumerate() {
            // println!("{}", r);
            // let rd = String::new(r);
            // let rd = r as &str;
            // let rd = "@" as str;
            if *r == "@" {
                map[i][j] = 0;
            }
        }
        println!("{}", line)
    }
    for (n, i) in map.iter().enumerate() {
        println!("{:?}", i);
    }
    return map;
}
