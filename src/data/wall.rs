use crate::utils::Vec3;
use crate::utils::Point;

#[derive(Debug, Clone)]
pub struct Wall {
    pub id: i64,
    pub a: Point,
    pub b: Point,
}

impl Wall {
    pub fn new(id: i64, a: Point, b: Point) -> Self {
        Self {
            id,
            a,
            b,
        }
    }
}

use crate::raw_data::raw_wall::RawWall;

impl From<RawWall> for Wall {
    fn from(raw_wall: RawWall) -> Self {
        Self {
            id: raw_wall.id,
            a: <Vec3 as Into<Point>>::into(raw_wall.start).snap(),
            b: <Vec3 as Into<Point>>::into(raw_wall.end).snap(),
        }
    }
}
