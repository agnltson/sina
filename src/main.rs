use std::{env, fs, process, thread::sleep, time::Duration};
use std::io::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, Point2};
use rerun::{RecordingStreamBuilder, Points2D};

mod parser;
mod raw_data;
mod data;
mod room_graph;
mod room_topology;
mod room_cdt;
mod navmesh;
mod navgraph;
mod utils;

use crate::parser::raw_data::parse_raw_data;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    if env::args().len() < 2 {
        println!("Missing room semantic file id");
        return Ok(());
    }
    let file_id = env::args().nth(1).unwrap();

    let prefix = String::from("input/");
    let suffix = String::from("/ase_scene_language.txt");
    let filepath = prefix + &file_id.as_str() + &suffix;

    let navgraph = navgraph::NavGraph::new(&filepath);
    let astar = navgraph.astar(0, 30);


    let rec = RecordingStreamBuilder::new("pathfinding")
        .spawn()?;

    let _ = navgraph.render_rerun(&rec);
    let _ = navgraph.render_rerun_path(&astar.unwrap(), &rec);
    Ok(())
}
