use std::{fs, process};
use std::io::prelude::*;
use macroquad::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, Point2};

mod parser;
mod room;
mod utils;

use crate::parser::room::parse_room;

#[macroquad::main("BasicShapes")]
async fn main() {
    let filepath = "input/room_sem2.txt";
    let mut file = fs::File::open(filepath).unwrap_or_else( |e| { eprintln!("{}: '{}'", e, filepath); process::exit(1) });
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);
    let room = parse_room(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
    println!("parsed room data:");
    println!("nb wall: {}", room.walls.len());
    println!("nb doors: {}", room.doors.len());
    println!("nb bboxes: {}", room.bboxes.len());

    let mut cdt: ConstrainedDelaunayTriangulation::<Point2<_>> = room.clone().into();

    loop {
        clear_background(BLACK);

        utils::cdt::cdt_render(&cdt);
        room.render();

        next_frame().await
    }
}
