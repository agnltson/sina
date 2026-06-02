use crate::utils::vec3::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct Bbox {
    id: u64,
    pub position: Vec3,
    pub angle: f32,
    pub scale: Vec3,
}

impl Bbox {
    pub fn new(id: u64, position: Vec3, angle: f32, scale: Vec3) -> Self {
        Self {
            id,
            position,
            angle,
            scale,
        }
    }

    pub fn ground_corners(&self) -> [(f32, f32); 4] {
        let hx = self.scale.x * 0.5;
        let hy = self.scale.y * 0.5;

        let c = self.angle.cos();
        let s = self.angle.sin();

        let local_corners = [
            (-hx, -hy),
            (-hx,  hy),
            ( hx,  hy),
            ( hx, -hy),
        ];

        local_corners.map(|(x, y)| {
            (self.position.x + x * c - y * s, self.position.y + x * s + y * c )
        })
    }
}
