#[derive(Debug, Copy, Clone)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x: x,
            y: y,
            z: z,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl Point {
    pub fn new((x, y): (f32, f32)) -> Self {
        Self {
            x,
            y,
        }
    }

    const EPS: f32 = 0.01;

    pub fn snap(self) -> Self {
        Self {
            x: (self.x / Self::EPS).round() * Self::EPS,
            y: (self.y / Self::EPS).round() * Self::EPS,
        }
    }
}


impl From<Vec3> for Point {
    fn from(vec: Vec3) -> Self {
        Point::new((vec.x, vec.y))
    }
}

impl From<(f32, f32)> for Point {
    fn from(xy: (f32, f32)) -> Self {
        Point::new(xy)
    }
}
