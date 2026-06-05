use std::{fs, process, thread::sleep, time::Duration};
use std::io::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, Point2};
use rerun::{RecordingStreamBuilder, Points2D};

mod parser;
mod raw_data;
mod data;
mod room_graph;
mod room_cdt;
mod room_topology;
mod utils;

use crate::parser::raw_data::parse_raw_data;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let filepath = "input/room_sem5.txt";
    let mut file = fs::File::open(filepath).unwrap_or_else( |e| { eprintln!("{}: '{}'", e, filepath); process::exit(1) });
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);

    let room_raw_data = parse_raw_data(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
    let room_data: data::Data = room_raw_data.into();

    let walls = room_data.extract_walls();
    let bboxes = room_data.extract_bboxes();

    let topo: room_topology::RoomTopology = room_data.into();
    let cdt: room_cdt::RoomCDT = topo.into();


    let rec = RecordingStreamBuilder::new("pathfinding")
        .spawn()?;

    let _ = cdt.render_rerun(&rec);
    Ok(())
}
