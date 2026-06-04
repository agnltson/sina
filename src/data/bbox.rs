use ordered_float::OrderedFloat;

use crate::utils::Vec3;
use crate::utils::Point;

#[derive(Debug, Clone)]
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

    pub fn into_polygon(&self) -> Vec<Point> {
        let hx = self.size.0 * 0.5;
        let hy = self.size.1 * 0.5;

        let c = self.angle.cos();
        let s = self.angle.sin();

        let local_corners = [
            (-hx, -hy),
            (-hx,  hy),
            ( hx,  hy),
            ( hx, -hy),
        ];

        local_corners.into_iter().map(|(x, y)| {
            Point::new((self.center.x + x * c - y * s, self.center.y + x * s + y * c ))
        }).collect()
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
