use crate::utils::vec3::Vec3;

#[derive(Debug)]
pub struct Door {
    id: u64,
    wall0_id: u64,
    wall1_id: u64,
    position: Vec3,
    width: f64,
    height: f64,
}

impl Door {
    pub fn new(id: u64, wall0_id: u64, wall1_id: u64, position: Vec3, width: f64, height: f64) -> Self {
        Self {
            id,
            wall0_id,
            wall1_id,
            position,
            width,
            height,
        }
    }
}
