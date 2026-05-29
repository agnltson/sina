mod parser;
mod room;
mod utils;

use crate::parser::wall::parse_wall;

fn main() {
    let mut input = "make_wall, id=0, a_x=-2.5483617782592773, a_y=-3.3396360874176025, a_z=0.021038055419921875, b_x=2.4016380310058594, b_y=-3.2896361351013184, b_z=0.021038055419921875, height=3.24, thickness=0.0\n";

    println!("parsed: {:?}", parse_wall(&mut input));
}
