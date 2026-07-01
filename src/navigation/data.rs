pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use super::data::wall::Wall;
use super::data::door::Door;
use super::data::bbox::BBox;
use super::raw_data::RawData;

// After clean up and projection from raw data
#[derive(Debug)]
pub struct Data {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<BBox>,
}

use rerun::{Color, LineStrips2D, RecordingStream};

impl Data {
    pub fn log(
        &self,
        rec: &RecordingStream,
        log_path: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {

        // =========================================================
        // WALLS
        // =========================================================
        let wall_lines: Vec<Vec<[f32; 2]>> = self.walls
            .iter()
            .map(|w| vec![
                [w.a.x.into_inner(), w.a.y.into_inner()],
                [w.b.x.into_inner(), w.b.y.into_inner()],
            ])
            .collect();

        rec.log(
            format!("{}/room/walls", log_path).as_str(),
            &LineStrips2D::new(wall_lines)
                .with_colors([Color::from_rgb(80, 120, 255)]),
        )?;

        // =========================================================
        // DOORS (aligned to walls)
        // =========================================================
        let mut door_lines = Vec::new();

        for door in &self.doors {
            let wall = self.walls
                .iter()
                .find(|w| w.id == door.wall_id);

            let Some(wall) = wall else { continue };

            let dx = wall.b.x.into_inner() - wall.a.x.into_inner();
            let dy = wall.b.y.into_inner() - wall.a.y.into_inner();

            let len = (dx * dx + dy * dy).sqrt();
            if len == 0.0 {
                continue;
            }

            let ux = dx / len;
            let uy = dy / len;

            let hw = door.width.into_inner() * 0.5;

            let cx = door.pos.x.into_inner();
            let cy = door.pos.y.into_inner();

            let ax = cx - ux * hw;
            let ay = cy - uy * hw;

            let bx = cx + ux * hw;
            let by = cy + uy * hw;

            door_lines.push(vec![
                [ax, ay],
                [bx, by],
            ]);
        }

        rec.log(
            format!("{}/room/doors", log_path).as_str(),
            &LineStrips2D::new(door_lines)
                .with_colors([Color::from_rgb(0, 200, 0)]),
        )?;

        // =========================================================
        // BBOXES (closed polygons)
        // =========================================================
        let mut bbox_strips = Vec::new();

        for bbox in &self.bboxes {
            let hx = bbox.size.0.into_inner() * 0.5;
            let hy = bbox.size.1.into_inner() * 0.5;

            let c = bbox.angle.into_inner().cos();
            let s = bbox.angle.into_inner().sin();

            let cx = bbox.center.x.into_inner();
            let cy = bbox.center.y.into_inner();

            let corners = [
                (-hx, -hy),
                (-hx,  hy),
                ( hx,  hy),
                ( hx, -hy),
            ];

            let mut poly: Vec<[f32; 2]> = corners
                .iter()
                .map(|(x, y)| {
                    let rx = x * c - y * s;
                    let ry = x * s + y * c;

                    [cx + rx, cy + ry]
                })
                .collect();

            // close loop
            poly.push(poly[0]);

            bbox_strips.push(poly);
        }

        rec.log(
            format!("{}/room/bboxes", log_path).as_str(),
            &LineStrips2D::new(bbox_strips)
                .with_colors([Color::from_rgb(255, 80, 80)]),
        )?;

        Ok(())
    }

    pub fn extract_bboxes(&self) -> Vec<BBox> {
        self.bboxes.clone()
    }

    pub fn extract_walls(&self) -> Vec<Wall> {
        self.walls.clone()
    }
}

impl From<RawData> for Data {

    fn from(raw_data: RawData) -> Self {
        Self {
            walls: raw_data.walls.iter().map(|w| (*w).into()).collect(),
            doors: raw_data.doors.iter().map(|d| (*d).into()).collect(),
            bboxes: raw_data.bboxes.iter().map(|b| (*b).into()).collect(),
        }
    }
}
