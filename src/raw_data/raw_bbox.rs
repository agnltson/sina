use ordered_float::OrderedFloat;

use crate::utils::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct RawBBox {
    id: usize,
    pub center: Vec3,
    pub angle: OrderedFloat<f32>,
    pub size: Vec3,
}

impl RawBBox {
    pub fn new(id: usize, center: Vec3, angle: OrderedFloat<f32>, size: Vec3) -> Self {
        Self {
            id,
            center,
            angle,
            size,
        }
    }
}
