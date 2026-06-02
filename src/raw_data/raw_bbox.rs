use crate::utils::Vec3;

#[derive(Debug, Copy, Clone)]
pub struct RawBBox {
    id: u64,
    pub center: Vec3,
    pub angle: f32,
    pub size: Vec3,
}

impl RawBBox {
    pub fn new(id: u64, center: Vec3, angle: f32, size: Vec3) -> Self {
        Self {
            id,
            center,
            angle,
            size,
        }
    }

    pub fn ground_corners(&self) -> [(f32, f32); 4] {
        let hx = self.size.x * 0.5;
        let hy = self.size.y * 0.5;

        let c = self.angle.cos();
        let s = self.angle.sin();

        let local_corners = [
            (-hx, -hy),
            (-hx,  hy),
            ( hx,  hy),
            ( hx, -hy),
        ];

        local_corners.map(|(x, y)| {
            (self.center.x + x * c - y * s, self.center.y + x * s + y * c )
        })
    }
}
