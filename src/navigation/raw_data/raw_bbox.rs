use ordered_float::OrderedFloat;

use crate::navigation::utils::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct RawBBox {
    pub center: Vec3,
    pub angle: OrderedFloat<f32>,
    pub size: Vec3,
}

impl RawBBox {
    pub fn new(center: Vec3, angle: OrderedFloat<f32>, size: Vec3) -> Self {
        Self {
            center,
            angle,
            size,
        }
    }
}
