use std::ops::{Sub, Add, Mul, Div};
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

    const EPS: OrderedFloat<f32> = OrderedFloat(0.05);

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

impl Into<(f64, f64)> for Point {
    fn into(self) -> (f64, f64) {
        (self.x.into_inner() as f64, self.y.into_inner() as f64)
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
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Add for Point {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl PartialOrd for Point {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Point {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.x.cmp(&other.x).then(self.y.cmp(&other.y))
    }
}

#[derive(Debug, Clone)]
pub struct Polygon {
    pub vertices: Vec<Point>,
}

impl Polygon {
    fn new(vertices: Vec<Point>) -> Self {
        Self { vertices }
    }
}

impl Polygon {
    pub fn contains(&self, point: Point) -> bool {
        let n = self.vertices.len();
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let vi = self.vertices[i];
            let vj = self.vertices[j];
            let xi = vi.x.into_inner();
            let yi = vi.y.into_inner();
            let xj = vj.x.into_inner();
            let yj = vj.y.into_inner();
            let px = point.x.into_inner();
            let py = point.y.into_inner();
            if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    pub fn centroid(&self) -> Point {
        let n = self.vertices.len() as f32;
        let x = self.vertices.iter().map(|v| v.x.into_inner()).sum::<f32>() / n;
        let y = self.vertices.iter().map(|v| v.y.into_inner()).sum::<f32>() / n;
        Point {
            x: OrderedFloat(x),
            y: OrderedFloat(y),
        }
    }

    pub fn intersect(&self, segment: (Point, Point)) -> bool {
        let n = self.vertices.len();

        for i in 0..n {
            let a = segment.0;
            let b = segment.1;

            let c = self.vertices[i];
            let d = self.vertices[(i + 1) % n];

            if orient(a, b, c) * orient(a, b, d) <= 0.0 &&
               orient(c, d, a) * orient(c, d, b) <= 0.0
            {
                return true;
            }
        }

        false
    }

    pub fn area(&self) -> f32 {
        let n = self.vertices.len();
        let mut sum = 0.0f32;
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % n];
            sum += a.x.into_inner() * b.y.into_inner();
            sum -= b.x.into_inner() * a.y.into_inner();
        }
        sum.abs() / 2.0
    }

    pub fn min_altitude(&self) -> f32 {
        let v = &self.vertices;
        assert!(v.len() == 3, "only for triangles");
        let a = v[0]; let b = v[1]; let c = v[2];
        let area = self.area();
        // altitude = 2 * area / base
        let ab = (b - a).length().into_inner();
        let bc = (c - b).length().into_inner();
        let ca = (a - c).length().into_inner();
        let alt_a = 2.0 * area / bc;
        let alt_b = 2.0 * area / ca;
        let alt_c = 2.0 * area / ab;
        alt_a.min(alt_b).min(alt_c)
    }
}

fn orient(p: Point, q: Point, r: Point) -> f32 {
    (q.x.into_inner() - p.x.into_inner())*(r.y.into_inner() - p.y.into_inner()) -
    (q.y.into_inner() - p.y.into_inner())*(r.x.into_inner() - p.x.into_inner())
}
