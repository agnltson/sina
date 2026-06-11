use ordered_float::OrderedFloat;

use crate::navigation::utils::Vec3;
use crate::navigation::utils::Point;
use crate::navigation::raw_data::raw_door::RawDoor;

#[derive(Debug, Clone)]
pub struct Door {
    pub pos: Point,
    pub width: OrderedFloat<f32>,
    pub wall_id: i64,
}

impl Door {
    pub fn new(pos: Point, width: OrderedFloat<f32>, wall_id: i64) -> Self {
        Self {
            pos,
            width,
            wall_id,
        }
    }
}

impl From<RawDoor> for Door {
    fn from(raw_door: RawDoor) -> Self {
        Self {
            pos: <Vec3 as Into<Point>>::into(raw_door.position).snap(),
            width: raw_door.width,
            wall_id: raw_door.wall0_id,
        }
    }
}
