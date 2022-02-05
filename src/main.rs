mod datatype;
mod map_reader;
mod multi_agent;
mod single_agent;

use multi_agent::cbs;

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

    /// Turn on conflict graph heuristic
    #[clap(short, long)]
    cg: bool,

    /// Turn on dependency graph heuristic
    #[clap(short, long)]
    dg: bool,

    /// Turn on weighted dependency graph heuristic
    #[clap(short, long)]
    wdg: bool,
    // /// Turn on heuristic(s): cg, dg, and/or wdg
    // #[clap(short, long, multiple_values = true)]
    // heuristics: Vec<heuristic::Heuristic>,
}

fn main() {
    let args = Args::parse();

    let map_instance = map_reader::map::import_map_instance(&args.file).unwrap();
    let map = map_instance.map;
    let starts = map_instance.starts;
    let goals = map_instance.goals;

    let heuristics: Vec<bool> = vec![args.cg, args.dg, args.wdg];

    // for r in &map {
    //     println!("{:?}", r);
    // }
    // println!("{:?}", starts);
    // println!("{:?}", goals);
    // println!();

    let paths = cbs::cbs(&map, starts, goals, None, args.disjoint, heuristics);
    match paths {
        Some(_paths) => {}
        None => println!("No solution."),
    }
}
