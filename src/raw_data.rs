pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use crate::raw_data::wall::Wall;
use crate::raw_data::door::Door;
use crate::raw_data::bbox::Bbox;

#[derive(Debug, Clone)]
pub struct RawData {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<Bbox>,
}

impl RawData {
    pub fn new(walls: Vec<Wall>, doors: Vec<Door>, bboxes: Vec<Bbox>) -> Self {
        Self {
            walls: walls,
            doors: doors,
            bboxes: bboxes,
        }
    }
}
