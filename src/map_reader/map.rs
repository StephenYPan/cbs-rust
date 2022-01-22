use crate::datatype::map_instance;
use crate::datatype::vertex;
use std::fs::File;
use std::io::{self, BufRead};

pub fn import_map_instance(filename: &str) -> Result<map_instance::MapInstance, std::io::Error> {
    let file = File::open(filename)?;
    let mut buff_reader = io::BufReader::new(file);
    let mut line = String::new();
    buff_reader.read_line(&mut line)?;
    // map instance
    let map_size: Vec<&str> = line.split_whitespace().collect();
    let row_size = map_size[0].parse::<usize>().unwrap();
    let col_size = map_size[1].parse::<usize>().unwrap();
    let mut map: Vec<Vec<u8>> = vec![vec![1; col_size]; row_size];
    for map_row in &mut map {
        line.clear();
        buff_reader.read_line(&mut line)?;
        let content: Vec<&str> = line.split_whitespace().collect();
        for (i, s) in content.iter().enumerate() {
            if *s == "@" {
                map_row[i] = 0;
            }
        }
    }
    // agents
    line.clear();
    buff_reader.read_line(&mut line)?;
    let content: Vec<&str> = line.split_whitespace().collect();
    let num_agents = content[0].parse::<usize>().unwrap();
    let mut starts: Vec<vertex::Vertex> = Vec::with_capacity(num_agents);
    let mut goals: Vec<vertex::Vertex> = Vec::with_capacity(num_agents);
    for _ in 0..num_agents {
        line.clear();
        buff_reader.read_line(&mut line)?;
        let content: Vec<&str> = line.split_whitespace().collect();
        let start = vertex::Vertex(
            content[0].parse::<u16>().unwrap(),
            content[1].parse::<u16>().unwrap(),
        );
        let goal = vertex::Vertex(
            content[2].parse::<u16>().unwrap(),
            content[3].parse::<u16>().unwrap(),
        );
        starts.push(start);
        goals.push(goal);
    }
    Ok(map_instance::MapInstance { map, starts, goals })
}
