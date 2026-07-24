use ordered_float::OrderedFloat;
use super::Point;

#[derive(Debug)]
pub struct Path {
    pos: Vec<Point>,
}

impl Path {

    pub fn points(&self) -> &[Point] {
        &self.pos
    }

    pub fn len(&self) -> usize {
        self.pos.len()
    }

    pub fn from_points(points: Vec<Point>) -> Self {
        Self {
            pos: points,
        }
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
}
