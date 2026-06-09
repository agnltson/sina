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

    let mut file = fs::File::open(&filepath).unwrap_or_else( |e| { eprintln!("{}: '{}'", e, filepath); process::exit(1) });
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);

    let room_raw_data = parse_raw_data(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
    let room_data: data::Data = room_raw_data.into();

    let topo: room_topology::RoomTopology = (&room_data).into();
    let navmesh: navmesh::NavMesh = (&topo).into();
    let navgraph: navgraph::NavGraph = (&navmesh).into();
    let astar = navgraph.astar(0, 27);


    let rec = RecordingStreamBuilder::new("pathfinding")
        .spawn()?;

    let _ = room_data.render_rerun(&rec);
    let _ = topo.render_rerun(&rec);
    let _ = navmesh.render_rerun(&rec);
    let _ = navgraph.render_rerun(&rec);
    let _ = navgraph.render_rerun_path(&astar.unwrap(), &rec);
    Ok(())
}
