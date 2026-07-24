pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use ordered_float::OrderedFloat;

use super::{
    utils::Point,
    data::{
        wall::Wall,
        door::Door,
        bbox::BBox,
    },
    raw_data::RawData,
};

// After clean up and projection from raw data
#[derive(Debug)]
pub struct Data {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<BBox>,
}

impl Data {
    pub fn extract_bboxes(&self) -> Vec<BBox> {
        self.bboxes.clone()
    }

    pub fn extract_walls(&self) -> Vec<Wall> {
        self.walls.clone()
    }

    pub fn wall_segments(&self) -> Vec<(Point, Point)> {
        self.walls.iter().map(|w| (w.a, w.b)).collect()
    }

    pub fn door_segments(&self) -> Vec<(Point, Point)> {
        let mut segments = Vec::new();

        for door in &self.doors {
            let Some(wall) = self.walls.iter().find(|w| w.id == door.wall_id) else { continue };

            let dx = wall.b.x.into_inner() - wall.a.x.into_inner();
            let dy = wall.b.y.into_inner() - wall.a.y.into_inner();
            let len = (dx * dx + dy * dy).sqrt();
            if len == 0.0 { continue; }

            let ux = dx / len;
            let uy = dy / len;
            let hw = door.width.into_inner() * 0.5;

            let cx = door.pos.x.into_inner();
            let cy = door.pos.y.into_inner();

            let a = Point { x: (cx - ux * hw).into(), y: (cy - uy * hw).into() };
            let b = Point { x: (cx + ux * hw).into(), y: (cy + uy * hw).into() };
            segments.push((a, b));
        }

        segments
    }

    pub fn bbox_polygons(&self) -> Vec<Vec<Point>> {
        self.bboxes.iter().map(|bbox| {
            let hx = bbox.size.0.into_inner() * 0.5;
            let hy = bbox.size.1.into_inner() * 0.5;
            let c = bbox.angle.into_inner().cos();
            let s = bbox.angle.into_inner().sin();
            let cx = bbox.center.x.into_inner();
            let cy = bbox.center.y.into_inner();

            [(-hx, -hy), (-hx, hy), (hx, hy), (hx, -hy)]
                .iter()
                .map(|(x, y)| {
                    let rx = x * c - y * s;
                    let ry = x * s + y * c;
                    Point { x: (cx + rx).into(), y: (cy + ry).into() }
                })
                .collect()
        }).collect()
    }
}

const MAX_HEIGHT_ABOVE_WALL_BASE: OrderedFloat<f32> = OrderedFloat(2.0);

impl From<RawData> for Data {
    fn from(raw_data: RawData) -> Self {
        let highest_wall_base = raw_data.walls
            .iter()
            .map(|w| w.start.z.max(w.end.z))
            .min()
            .unwrap_or(OrderedFloat(0.0));

        let bboxes: Vec<BBox> = raw_data.bboxes
            .iter()
            .filter(|b| {
                let bbox_base = b.center.z - b.size.z * OrderedFloat(0.5);
                bbox_base <= highest_wall_base + MAX_HEIGHT_ABOVE_WALL_BASE
            })
            .cloned()
            .map(Into::into)
            .collect();

        Self {
            walls: raw_data.walls.iter().map(|w| (*w).into()).collect(),
            doors: raw_data.doors.iter().map(|d| (*d).into()).collect(),
            bboxes,
        }
    }
}
