use ordered_float::OrderedFloat;

use crate::utils::Vec3;
use crate::utils::Point;

pub struct BBox {
    pub center: Point,
    pub size: (OrderedFloat<f32>, OrderedFloat<f32>),
    pub angle: OrderedFloat<f32>,
}

impl BBox {
    pub fn new(center: Point, size: (OrderedFloat<f32>, OrderedFloat<f32>), angle: OrderedFloat<f32>) -> Self {
        Self {
            center,
            size,
            angle,
        }
    }
}

use crate::raw_data::raw_bbox::RawBBox;

impl From<RawBBox> for BBox {
    fn from(raw_bbox: RawBBox) -> Self {
        let size_snap: Point = <Vec3 as Into<Point>>::into(raw_bbox.size).snap();
        Self {
            center: <Vec3 as Into<Point>>::into(raw_bbox.center).snap(),
            size: (size_snap.x, size_snap.y),
            angle: raw_bbox.angle,
        }
    }
}
