use crate::utils::vec3::Vec3;

#[derive(Debug)]
pub struct Door {
    id: u64,
    a: Vec3,
    b: Vec3,
}

impl Door {
    pub fn new(id: u64, a: Vec3, b: Vec3) -> Self {
        Self {
            id,
            a,
            b,
        }
    }
}
