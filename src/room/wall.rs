use spade::{Point2};

use crate::utils::vec3::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct Wall {
    pub id: u64,
    pub start: Vec3,
    pub end: Vec3,
}

impl Wall {
    pub fn new(id: u64, start: Vec3, end: Vec3) -> Self {
        Self {
            id,
            start,
            end,
        }
    }
}
