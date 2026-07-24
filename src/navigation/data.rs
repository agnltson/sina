pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use super::data::wall::Wall;
use super::data::door::Door;
use super::data::bbox::BBox;
use super::raw_data::RawData;
use ordered_float::OrderedFloat;

// After clean up and projection from raw data
#[derive(Debug)]
pub struct Data {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<BBox>,
}

impl Data {
    pub fn extract_bboxes(&self) -> Vec<BBox> {
        self.bboxes.clone()
    }

    pub fn extract_walls(&self) -> Vec<Wall> {
        self.walls.clone()
    }
}

const MAX_HEIGHT_ABOVE_WALL_BASE: OrderedFloat<f32> = OrderedFloat(2.0);

impl From<RawData> for Data {
    fn from(raw_data: RawData) -> Self {
        let highest_wall_base = raw_data.walls
            .iter()
            .map(|w| w.start.z.max(w.end.z))
            .min()
            .unwrap_or(OrderedFloat(0.0));

        let bboxes: Vec<BBox> = raw_data.bboxes
            .iter()
            .filter(|b| {
                let bbox_base = b.center.z - b.size.z * OrderedFloat(0.5);
                bbox_base <= highest_wall_base + MAX_HEIGHT_ABOVE_WALL_BASE
            })
            .cloned()
            .map(Into::into)
            .collect();

        Self {
            walls: raw_data.walls.iter().map(|w| (*w).into()).collect(),
            doors: raw_data.doors.iter().map(|d| (*d).into()).collect(),
            bboxes,
        }
    }
}
