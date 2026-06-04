use std::{fs, process, thread::sleep, time::Duration};
use std::io::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, Point2};
use rerun::{RecordingStreamBuilder, Points2D};

mod parser;
mod raw_data;
mod data;
mod room_graph;
mod room_cdt;
mod obstacles;
mod utils;

use crate::parser::raw_data::parse_raw_data;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let filepath = "input/room_sem2.txt";
    let mut file = fs::File::open(filepath).unwrap_or_else( |e| { eprintln!("{}: '{}'", e, filepath); process::exit(1) });
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);

    let room_raw_data = parse_raw_data(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
    let room_data: data::Data = room_raw_data.into();

    let walls = room_data.extract_walls();
    let bboxes = room_data.extract_bboxes();
    let bboxes_polygons: Vec<Vec<_>> = bboxes.iter().map(|b| b.into_polygon()).collect();

    let obstacles = obstacles::Obstacles::from_clipping(bboxes_polygons, &walls);

    let room_graph: room_graph::RoomGraph = room_data.into();
    let mut cdt: room_cdt::RoomCDT = room_graph.into();

    cdt.add_obstacles(&obstacles);

    let rec = RecordingStreamBuilder::new("pathfinding")
        .spawn()?;

    let _ = cdt.render_rerun(&rec);
    //let _ = obstacles.render_rerun(&rec);
    Ok(())
}
