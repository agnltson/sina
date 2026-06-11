use crate::navigation::utils::Vec3;
use crate::navigation::utils::Point;
use crate::navigation::raw_data::raw_wall::RawWall;

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

impl From<RawWall> for Wall {
    fn from(raw_wall: RawWall) -> Self {
        Self {
            id: raw_wall.id,
            a: <Vec3 as Into<Point>>::into(raw_wall.start).snap(),
            b: <Vec3 as Into<Point>>::into(raw_wall.end).snap(),
        }
    }
}
