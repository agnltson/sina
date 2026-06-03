use ordered_float::OrderedFloat;

use crate::utils::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct RawDoor {
    pub id: usize,
    pub wall0_id: usize,
    wall1_id: usize,
    pub position: Vec3,
    pub width: OrderedFloat<f32>,
    height: OrderedFloat<f32>,
}

impl RawDoor {
    pub fn new(id: usize, wall0_id: usize, wall1_id: usize, position: Vec3, width: OrderedFloat<f32>, height: OrderedFloat<f32>) -> Self {
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
