use ordered_float::OrderedFloat;

#[derive(Debug, Copy, Clone)]
pub struct Vec3 {
    pub x: OrderedFloat<f32>,
    pub y: OrderedFloat<f32>,
    pub z: OrderedFloat<f32>,
}

impl Vec3 {
    pub fn new(x: OrderedFloat<f32>, y: OrderedFloat<f32>, z: OrderedFloat<f32>) -> Self {
        Self {
            x: x,
            y: y,
            z: z,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Point {
    pub x: OrderedFloat<f32>,
    pub y: OrderedFloat<f32>,
}

use std::hash::{Hash, Hasher};

impl Hash for Point {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.to_bits().hash(state);
        self.y.to_bits().hash(state);
    }
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.x.to_bits() == other.x.to_bits() && self.y.to_bits() == other.y.to_bits()
    }
}

impl Eq for Point {}

impl Point {
    pub fn new((x, y): (OrderedFloat<f32>, OrderedFloat<f32>)) -> Self {
        Self {
            x,
            y,
        }
    }

    const EPS: OrderedFloat<f32> = OrderedFloat(0.01);

    pub fn snap(self) -> Self {
        Self {
            x: OrderedFloat((self.x / Self::EPS).round()) * Self::EPS,
            y: OrderedFloat((self.y / Self::EPS).round()) * Self::EPS,
        }
    }
}


impl From<Vec3> for Point {
    fn from(vec: Vec3) -> Self {
        Point::new((vec.x, vec.y))
    }
}

impl From<(OrderedFloat<f32>, OrderedFloat<f32>)> for Point {
    fn from(xy: (OrderedFloat<f32>, OrderedFloat<f32>)) -> Self {
        Point::new(xy)
    }
}
