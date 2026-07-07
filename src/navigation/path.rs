use std::{
    fs::File,
    io::{
        BufRead,
        BufReader,
    }
};
use rerun::{Points2D, LineStrips2D, RecordingStream, Color};
use ordered_float::OrderedFloat;
use super::Point;

#[derive(Debug)]
pub struct Path {
    pos: Vec<Point>,
}

impl Path {
    pub fn from_points(points: Vec<Point>) -> Self {
        Self {
            pos: points,
        }
    }

    pub fn from_closed_loop(filepath: &str) -> Self {
        let source_name = "trajectory.csv";
        let full_path = format!("{}/{}", filepath, source_name);

        let file = File::open(&full_path)
            .unwrap_or_else(|e| panic!("Unable to open {}: {}", full_path, e));
        let reader = BufReader::new(file);

        let mut pos = Vec::new();

        for (i, line) in reader.lines().enumerate() {
            let line = line.unwrap_or_else(|e| panic!("Reading error line {}: {}", i, e));

            if i == 0 {
                continue;
            }

            let fields: Vec<&str> = line.split(',').collect();

            let x: f64 = fields[3]
                .parse()
                .unwrap_or_else(|e| panic!("Error while parsing x line {}: {}", i, e));
            let y: f64 = fields[4]
                .parse()
                .unwrap_or_else(|e| panic!("Error while parsing y line {}: {}", i, e));

            pos.push((x, y).into());
        }

        Path { pos }
    }

    fn project_on_segment(pos: Point, a: Point, b: Point) -> (Point, f32) {
        let ab = b - a;
        let ap = pos - a;

        let len_sq = ab.dot(ab);
        let t = if len_sq > 1e-9 {
            (ap.dot(ab) / len_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let proj = Point {
            x: a.x + OrderedFloat(t) * ab.x,
            y: a.y + OrderedFloat(t) * ab.y,
        };

        (proj, t)
    }

    pub fn closest_segment(&self, pos: Point, hint: usize, window: usize) -> Option<(usize, f32, f32)> {
        if self.pos.len() < 2 {
            return None;
        }

        let lo = hint.saturating_sub(window);
        let hi = (hint + window).min(self.pos.len() - 2);

        (lo..=hi)
            .map(|i| {
                let (proj, t) = Self::project_on_segment(pos, self.pos[i], self.pos[i + 1]);
                (i, t, pos.dist_to(proj))
            })
            .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap())
    }

    pub fn lookahead_target(&self, idx: usize, t: f32, lookahead: f32) -> Point {
        let seg_len = self.pos[idx].dist_to(self.pos[idx + 1]);
        let mut remaining = lookahead - (1.0 - t) * seg_len;
        let mut i = idx + 1;

        while remaining > 0.0 && i + 1 < self.pos.len() {
            let d = self.pos[i].dist_to(self.pos[i + 1]);
            if d >= remaining {
                let ratio = OrderedFloat(remaining / d);
                let ab = self.pos[i + 1] - self.pos[i];
                return Point {
                    x: self.pos[i].x + ratio * ab.x,
                    y: self.pos[i].y + ratio * ab.y,
                };
            }
            remaining -= d;
            i += 1;
        }

        *self.pos.last().unwrap()
    }

    pub fn log(
        &self,
        rec: &RecordingStream,
        log_path: &str,
        ) -> anyhow::Result<()> {
        let points: Vec<[f32; 2]> = self
            .pos
            .iter()
            .map(|p| [p.x.into_inner(), p.y.into_inner()])
            .collect();

        rec.log(
            format!("{}/path/points", log_path).as_str(),
            &Points2D::new(points.clone()),
        )?;

        if points.len() >= 2 {
            rec.log(
                format!("{}/path/line", log_path).as_str(),
                &LineStrips2D::new(vec![points])
                    .with_colors([Color::from_rgb(255, 165, 0)]), // orange
            )?;
        }

        Ok(())
    }
}
