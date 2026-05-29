pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use crate::room::wall::Wall;
use crate::room::door::Door;
use crate::room::bbox::Bbox;

pub struct Room {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<bbox::Bbox>,
}

impl Room {
    pub fn new(walls: Vec<Wall>, doors: Vec<Door>, bboxes: Vec<Bbox>) -> Self {
        Self {
            walls,
            doors,
            bboxes,
        }
    }
}
