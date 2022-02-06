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
    #[clap(long)]
    disjoint: bool,

    /// Turn on conflict graph heuristic
    #[clap(long)]
    cg: bool,

    /// Turn on dependency graph heuristic
    #[clap(long)]
    dg: bool,

    /// Turn on weighted dependency graph heuristic
    #[clap(long)]
    wdg: bool,
}

fn main() {
    let args = Args::parse();

    let map_instance = map_reader::map::import_map_instance(&args.file).unwrap();
    let heuristics: Vec<bool> = vec![args.cg, args.dg, args.wdg];

    let paths = cbs::cbs(&map_instance, None, args.disjoint, heuristics);
    match paths {
        Some(_paths) => {}
        None => println!("No solution."),
    }
}
