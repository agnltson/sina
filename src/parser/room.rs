use winnow::Parser;

use crate::room::Room;

pub fn parse_room(input: &mut str) -> Room {
    Room::new(Vec::new(), Vec::new(), Vec::new())
}
