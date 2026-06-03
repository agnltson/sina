use std::{fs, process, thread::sleep, time::Duration};
use std::io::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, Point2};
use rerun::{RecordingStreamBuilder, Points2D};

mod parser;
mod raw_data;
mod data;
mod room_graph;
mod room_cdt;
mod utils;

use crate::parser::raw_data::parse_raw_data;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let filepath = "input/room_sem2.txt";
    let mut file = fs::File::open(filepath).unwrap_or_else( |e| { eprintln!("{}: '{}'", e, filepath); process::exit(1) });
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);

    let room_raw_data = parse_raw_data(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
    let room_data: data::Data = room_raw_data.into();
    let bboxes = room_data.extract_bboxes();
    let room_graph: room_graph::RoomGraph = room_data.into();

    let rec = RecordingStreamBuilder::new("pathfinding")
        .spawn()?;

    let _ = room_graph.render_rerun(&rec);

    let mut t: f32 = 0.0;

    loop {
        // ping-pong between 0 and 1
        let x = (t.sin() * 0.5 + 0.5) * 10.0;
        let y = 0.0;

        rec.log(
            "debug/dot",
            &Points2D::new([[x, y]]),
        )?;

        t += 0.05;

        sleep(Duration::from_millis(16));
    }
}
