use crate::utils::Vec3;
use crate::utils::Point;

pub struct Wall {
    pub a: Point,
    pub b: Point,
}

impl Wall {
    pub fn new(a: Point, b: Point) -> Self {
        Self {
            a,
            b,
        }
    }
}

use crate::raw_data::raw_wall::RawWall;

impl From<RawWall> for Wall {
    fn from(raw_wall: RawWall) -> Self {
        Self {
            a: <Vec3 as Into<Point>>::into(raw_wall.start).snap(),
            b: <Vec3 as Into<Point>>::into(raw_wall.end).snap(),
        }
    }
}
