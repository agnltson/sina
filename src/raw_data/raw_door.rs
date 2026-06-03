use ordered_float::OrderedFloat;

use crate::utils::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct RawDoor {
    pub id: u64,
    pub wall0_id: u64,
    wall1_id: u64,
    pub position: Vec3,
    pub width: OrderedFloat<f32>,
    height: OrderedFloat<f32>,
}

impl RawDoor {
    pub fn new(id: u64, wall0_id: u64, wall1_id: u64, position: Vec3, width: OrderedFloat<f32>, height: OrderedFloat<f32>) -> Self {
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
