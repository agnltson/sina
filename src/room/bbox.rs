use crate::utils::vec3::Vec3;

#[derive(Debug)]
pub struct Bbox {
    id: usize,
    position: Vec3,
    angle: f64,
    scale: Vec3,
}

impl Bbox {
    pub fn new(id: usize, position: Vec3, angle: f64, scale: Vec3) -> Self {
        Self {
            id,
            position,
            angle,
            scale,
        }
    }
}
