use std::{fs, process};
use std::io::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, Point2};
use rerun::{RecordingStreamBuilder, Points2D};

mod parser;
mod raw_data;
mod utils;

use crate::parser::raw_data::parse_raw_data;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let filepath = "input/room_sem1.txt";
    let mut file = fs::File::open(filepath).unwrap_or_else( |e| { eprintln!("{}: '{}'", e, filepath); process::exit(1) });
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);
    let room = parse_raw_data(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
    println!("parsed room data:");
    println!("nb wall: {}", room.walls.len());
    println!("nb doors: {}", room.doors.len());
    println!("nb bboxes: {}", room.bboxes.len());


    let rec = RecordingStreamBuilder::new("pathfinding")
        .spawn()?;

    Ok(())
}
