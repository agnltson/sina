use clipper2::{Clipper, FillRule, Paths};
use rerun::{LineStrips2D, RecordingStream};

use crate::data::{wall::Wall, bbox::BBox};
use crate::utils::{Point};

#[derive(Debug, Clone)]
pub struct Obstacles {
    // each element is a closed loop that defines a polygon
    pub vertices: Vec<Vec<Point>>,
}

impl Obstacles {
    pub fn from_clipping(object_vertices: Vec<Vec<Point>>, walls: &Vec<Wall>) -> Self {
        let polygons = order_walls_as_polygons(walls);

        let wall_paths: Paths = polygons
            .iter()
            .map(|polygon| {
                polygon
                    .iter()
                    .map(|w| (w.a.x.into_inner() as f64, w.a.y.into_inner() as f64))
                    .collect::<Vec<(f64, f64)>>()
            })
            .collect::<Vec<_>>()
            .into();

        let object_paths: Paths = object_vertices
            .iter()
            .map(|polygon| {
                polygon
                    .iter()
                    .map(|p| (p.x.into_inner() as f64, p.y.into_inner() as f64))
                    .collect::<Vec<(f64, f64)>>()
            })
            .collect::<Vec<_>>()
            .into();

        let result = Clipper::new()
            .add_subject(object_paths)
            .add_clip(wall_paths)
            .intersect(FillRule::NonZero)
            .unwrap();

        let vertices: Vec<Vec<Point>> = result
            .into_iter()
            .map(|path| {
                path.into_iter()
                    .map(|p| Point {
                        x: (p.x() as f32).into(),
                        y: (p.y() as f32).into(),
                    })
                    .collect()
            })
            .collect();

        Self { vertices }
    }

    pub fn render_rerun(
        &self,
        rec: &rerun::RecordingStream,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut strips = Vec::new();

        for poly in &self.vertices {
            if poly.len() < 2 {
                continue;
            }

            let mut pts: Vec<[f32; 2]> = poly
                .iter()
                .map(|p| [p.x.into_inner(), p.y.into_inner()])
                .collect();

            // close loop
            pts.push([
                poly[0].x.into_inner(),
                poly[0].y.into_inner(),
            ]);

            strips.push(pts);
        }

        rec.log(
            "obstacle",
            &rerun::LineStrips2D::new(strips),
        )?;

        Ok(())
    }
}

pub fn order_walls_as_polygons(walls: &Vec<Wall>) -> Vec<Vec<Wall>> {
    if walls.is_empty() {
        return vec![];
    }

    let mut remaining: Vec<Wall> = walls.clone();
    let mut polygons: Vec<Vec<Wall>> = Vec::new();

    while !remaining.is_empty() {
        let mut ordered: Vec<Wall> = Vec::new();
        ordered.push(remaining.remove(0));

        loop {
            let last_point = ordered.last().unwrap().b;
            let last_snapped = last_point.snap();

            if let Some(pos) = remaining
                .iter()
                .position(|w| w.a == last_point || w.a.snap() == last_snapped)
            {
                ordered.push(remaining.remove(pos));
            } else if let Some(pos) = remaining
                .iter()
                .position(|w| w.b == last_point || w.b.snap() == last_snapped)
            {
                let mut w = remaining.remove(pos);
                std::mem::swap(&mut w.a, &mut w.b);
                ordered.push(w);
            } else {
                // No continuation found — this cycle is complete (or broken)
                break;
            }
        }

        polygons.push(ordered);
    }

    polygons
}
