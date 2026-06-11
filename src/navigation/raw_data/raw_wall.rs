use crate::navigation::utils::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct RawWall {
    pub id: i64,
    pub start: Vec3,
    pub end: Vec3,
}

impl RawWall {
    pub fn new(id: i64, start: Vec3, end: Vec3) -> Self {
        Self {
            id,
            start,
            end,
        }
    }
}
