pub(crate) mod raw_wall;
pub(crate) mod raw_door;
pub(crate) mod raw_bbox;

#[derive(Debug, Clone)]
pub struct RawData {
    pub walls: Vec<raw_wall::RawWall>,
    pub doors: Vec<raw_door::RawDoor>,
    pub bboxes: Vec<raw_bbox::RawBBox>,
}

impl RawData {
    pub fn new(walls: Vec<raw_wall::RawWall>, doors: Vec<raw_door::RawDoor>, bboxes: Vec<raw_bbox::RawBBox>) -> Self {
        Self {
            walls: walls,
            doors: doors,
            bboxes: bboxes,
        }
    }
}
