use std::{fs, process};
use std::io::prelude::*;

mod parser;
mod room;
mod utils;

use crate::parser::room::parse_room;

fn main() {
    let filepath = "input/room_sem.txt";
    let mut file = fs::File::open(filepath).unwrap_or_else( |e| { eprintln!("{}: '{}'", e, filepath); process::exit(1) });
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);
    println!("parsed: {:?}", parse_room(&mut contents.trim()));
}
