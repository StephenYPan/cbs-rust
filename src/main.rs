mod datatype;
mod file_reader;
mod multi_agent;
mod single_agent;

use multi_agent::cbs::cbs;

use clap::Parser;

#[derive(Parser)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Path to map file
    // #[clap(short, long, multiple_values = true)]
    #[clap(short, long)]
    file: String,

    /// Turn on disjoint split
    #[clap(short, long)]
    disjoint: bool,
}

fn main() {
    let args = Args::parse();

    let map_instance = file_reader::map::import_map_instance(&args.file).unwrap();
    let map = map_instance.map;
    let starts = map_instance.starts;
    let goals = map_instance.goals;

    // for r in &map {
    //     println!("{:?}", r);
    // }
    // println!("{:?}", starts);
    // println!("{:?}", goals);
    // println!();

    let paths = cbs(&map, starts, goals, None, args.disjoint);
    match paths {
        Some(_paths) => {}
        None => println!("No solution."),
    }
}
