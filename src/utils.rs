pub(crate) mod vec3;
pub(crate) mod cdt;

use spade::Point2;

pub fn point2_add(u: Point2<f32>, v: Point2<f32>) -> Point2<f32> {
    Point2::new(u.x + v.x, u.y + v.y)
}

pub fn point2_sub(u: Point2<f32>, v: Point2<f32>) -> Point2<f32> {
    Point2::new(u.x - v.x, u.y - v.y)
}
