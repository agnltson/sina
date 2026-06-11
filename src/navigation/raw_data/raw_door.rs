use ordered_float::OrderedFloat;

use crate::navigation::utils::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct RawDoor {
    pub id: i64,
    pub wall0_id: i64,
    wall1_id: i64,
    pub position: Vec3,
    pub width: OrderedFloat<f32>,
    height: OrderedFloat<f32>,
}

impl RawDoor {
    pub fn new(id: i64, wall0_id: i64, wall1_id: i64, position: Vec3, width: OrderedFloat<f32>, height: OrderedFloat<f32>) -> Self {
        Self {
            id,
            wall0_id,
            wall1_id,
            position,
            width,
            height,
        }
    }
}
