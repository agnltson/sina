pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use crate::data::wall::Wall;
use crate::data::door::Door;
use crate::data::bbox::BBox;

// After clean up and projection from raw data
pub struct Data {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<BBox>,
}

use crate::raw_data::RawData;

impl From<RawData> for Data {
    fn from(raw_data: RawData) -> Self {
        Self {
            walls: raw_data.walls.iter().map(|w| (*w).into()).collect(),
            doors: raw_data.doors.iter().map(|d| (*d).into()).collect(),
            bboxes: raw_data.bboxes.iter().map(|b| (*b).into()).collect(),
        }
    }
}
