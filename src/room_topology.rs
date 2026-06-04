use ordered_float::OrderedFloat;
use rerun::{Color, LineStrips2D, RecordingStream};
use clipper2::{Clipper, FillRule, Path, Paths};

use crate::data::{Data, bbox::BBox};
use crate::room_graph::{RoomGraph, Edge};
use crate::utils::Point;

#[derive(Debug, Clone)]
struct Polygon {
    pub vertices: Vec<Point>,
}

impl Polygon {
    fn new(vertices: Vec<Point>) -> Self {
        Self {
            vertices,
        }
    }
}

impl From<Path> for Polygon {
    fn from(path: Path) -> Self {
        Polygon {
            vertices: path.into_iter()
                .map(|p| Point {
                    x: (p.x() as f32).into(),
                    y: (p.y() as f32).into(),
                })
                .collect(),
        }
    }
}

impl Into<Path> for Polygon {
    fn into(self) -> Path {
        self.vertices.iter().map(|p| (*p).into()).collect::<Vec<(f64, f64)>>().into()
    }
}

impl From<Vec<Point>> for Polygon {
    fn from(vertices: Vec<Point>) -> Self {
        Self {
            vertices,
        }
    }
}

impl From<BBox> for Polygon {
    fn from(bbox: BBox) -> Self {
        let hx = bbox.size.0 * 0.5;
        let hy = bbox.size.1 * 0.5;

        let c = bbox.angle.cos();
        let s = bbox.angle.sin();

        let local_corners = [
            (-hx, -hy),
            (-hx,  hy),
            ( hx,  hy),
            ( hx, -hy),
        ];

        Self {
            vertices: local_corners.into_iter().map(|(x, y)| {
                Point::new((bbox.center.x + x * c - y * s, bbox.center.y + x * s + y * c ))
                }).collect::<Vec<_>>().into(),
        }
    }
}

pub struct RoomTopology {
    pub borders: Vec<Polygon>,
    pub holes: Vec<Polygon>,
}

impl RoomTopology {

    fn new(borders: Vec<Polygon>, holes: Vec<Polygon>) -> Self {
        Self {
            borders,
            holes,
        }
    }

    pub fn render_rerun(
        &self,
        rec: &RecordingStream,
    ) -> Result<(), Box<dyn std::error::Error>> {

        // -------------------------
        // Borders
        // -------------------------

        let mut border_strips = Vec::new();

        for border in &self.borders {
            if border.vertices.is_empty() {
                continue;
            }

            let mut strip: Vec<[f32; 2]> = border
                .vertices
                .iter()
                .map(|p| [p.x.into_inner(), p.y.into_inner()])
                .collect();

            // close polygon
            strip.push([
                border.vertices[0].x.into_inner(),
                border.vertices[0].y.into_inner(),
            ]);

            border_strips.push(strip);
        }

        rec.log(
            "topology/borders",
            &LineStrips2D::new(border_strips)
                .with_colors([Color::from_rgb(0, 200, 255)]),
        )?;

        // -------------------------
        // Holes
        // -------------------------

        let mut hole_strips = Vec::new();

        for hole in &self.holes {
            if hole.vertices.is_empty() {
                continue;
            }

            let mut strip: Vec<[f32; 2]> = hole
                .vertices
                .iter()
                .map(|p| [p.x.into_inner(), p.y.into_inner()])
                .collect();

            // close polygon
            strip.push([
                hole.vertices[0].x.into_inner(),
                hole.vertices[0].y.into_inner(),
            ]);

            hole_strips.push(strip);
        }

        rec.log(
            "topology/holes",
            &LineStrips2D::new(hole_strips)
                .with_colors([Color::from_rgb(255, 100, 100)]),
        )?;

        Ok(())
    }
}

impl From<Data> for RoomTopology {
    fn from(data: Data) -> Self {
        let bboxes = data.bboxes.clone();
        let poly_bboxes: Vec<Polygon> = bboxes.iter().map(|b| b.clone().into()).collect();

        let room_graph: RoomGraph = data.into();
        let room_polygons = graph_into_polygons(&room_graph);
        let doors_polygons = extract_doors_as_polygons(&room_graph);
        let borders = clip_doors(room_polygons, doors_polygons);
        let holes = clip_holes(poly_bboxes, &borders);
        RoomTopology::new(borders, holes)
    }
}

fn clip_holes(holes: Vec<Polygon>, borders: &Vec<Polygon>) -> Vec<Polygon> {
    let holes_paths: Paths = holes.iter()
        .map(|p| <Polygon as Into<Path>>::into(p.clone()))
        .collect::<Vec<Path>>()
        .into();
    let border_paths: Paths = borders.iter()
        .map(|p| <Polygon as Into<Path>>::into(p.clone()))
        .collect::<Vec<Path>>()
        .into();
    let result = Clipper::new()
        .add_subject(holes_paths)
        .add_clip(border_paths)
        .intersect(FillRule::NonZero)
        .unwrap();
    result
        .into_iter()
        .map(|path| path.into())
        .collect()
}

fn clip_doors(borders: Vec<Polygon>, doors: Vec<Polygon>) -> Vec<Polygon> {
    let border_paths: Paths = borders.iter()
        .map(|p| <Polygon as Into<Path>>::into(p.clone()))
        .collect::<Vec<Path>>()
        .into();
    let doors_paths: Paths = doors.iter()
        .map(|p| <Polygon as Into<Path>>::into(p.clone()))
        .collect::<Vec<Path>>()
        .into();
    let result = Clipper::new()
        .add_subject(border_paths)
        .add_clip(doors_paths)
        .union(FillRule::NonZero)
        .unwrap();
    result
        .into_iter()
        .map(|path| path.into())
        .collect()
}

pub fn extract_doors_as_polygons(graph: &RoomGraph) -> Vec<Polygon> {
    if graph.edges.is_empty() {
        return vec![];
    }
    let door_frame_offset = OrderedFloat(0.15);
    let mut polygons = Vec::new();
    for edge in &graph.edges {
        if edge.doors.is_empty() { continue; }
        let unit_dir = (edge.b - edge.a).to_unit();
        let normal = Point { x: -unit_dir.y, y: unit_dir.x };
        let offset_vec = Point {
            x: normal.x * door_frame_offset,
            y: normal.y * door_frame_offset,
        };
        for door in &edge.doors {
            let half = door.width * OrderedFloat(0.5);
            let along = Point { x: unit_dir.x * half, y: unit_dir.y * half };
            let start = Point { x: door.pos.x - along.x, y: door.pos.y - along.y };
            let end   = Point { x: door.pos.x + along.x, y: door.pos.y + along.y };

            let corner1 = Point { x: start.x - offset_vec.x, y: start.y - offset_vec.y };
            let corner2 = Point { x: start.x + offset_vec.x, y: start.y + offset_vec.y };
            let corner3 = Point { x: end.x   + offset_vec.x, y: end.y   + offset_vec.y };
            let corner4 = Point { x: end.x   - offset_vec.x, y: end.y   - offset_vec.y };

            polygons.push(vec![corner1, corner2, corner3, corner4].into());
        }
    }
    polygons
}

pub fn graph_into_polygons(graph: &RoomGraph) -> Vec<Polygon> {
    if graph.edges.is_empty() {
        return vec![];
    }
    let mut remaining: Vec<Edge> = graph.edges.clone();
    let mut polygons: Vec<Polygon> = Vec::new();
    while !remaining.is_empty() {
        let mut ordered: Vec<Edge> = Vec::new();
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
                break;
            }
        }
        let vertices: Vec<Point> = ordered.iter().map(|e| e.a).collect();
        polygons.push(vertices.into());
    }
    polygons
}
