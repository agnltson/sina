use std::ops::{Sub, Add};
use ordered_float::OrderedFloat;
use spade::{Point2};

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

    pub fn dot(self, other: Self) -> OrderedFloat<f32> {
        self.x * other.x + self.y * other.y
    }

    pub fn length(self) -> OrderedFloat<f32> {
        OrderedFloat(self.dot(self).sqrt())
    }

    pub fn to_unit(self) -> Self {
        let len = self.length();
        Self {
            x: self.x / len,
            y: self.y / len,
        }
    }
}

impl Into<Point2<f32>> for Point {
    fn into(self) -> Point2<f32> {
        Point2::new(self.x.into_inner(), self.y.into_inner())
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

impl Sub for Point {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.y,
            y: self.y - other.y,
        }
    }
}

impl Add for Point {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.y,
            y: self.y + other.y,
        }
    }
}
