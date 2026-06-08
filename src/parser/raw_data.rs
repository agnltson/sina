use winnow::error::ModalResult;

use crate::raw_data::RawData;

use crate::parser::wall::parse_wall;
use crate::parser::door::parse_door;
use crate::parser::bbox::parse_bbox;
use crate::parser::skip::skip_window;

pub fn parse_raw_data(input: &mut &str) -> ModalResult<RawData> {
    let mut walls = Vec::new();
    let mut doors = Vec::new();
    let mut bboxes = Vec::new();

    loop {
        if input.is_empty() {
            break;
        }

        if let Ok(wall) = parse_wall(input) {
            walls.push(wall);
        } else if let Ok(door) = parse_door(input) {
            doors.push(door);
        } else if let Ok(bbox) = parse_bbox(input) {
            bboxes.push(bbox);
        } else if let Ok(()) = skip_window(input) {
        } else {
            panic!("Unknown line: {}", input);
        }
    }
    Ok(RawData::new(walls, doors, bboxes))
}
