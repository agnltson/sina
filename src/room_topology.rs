use ordered_float::OrderedFloat;
use rerun::{Color, LineStrips2D, RecordingStream};
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::single::SingleFloatOverlay;
use i_overlay::i_float::float::compatible::FloatPointCompatible;

use crate::data::{Data, bbox::BBox};
use crate::room_graph::{RoomGraph, Edge};
use crate::utils::{Point, Polygon};

impl From<Vec<Point>> for Polygon {
    fn from(vertices: Vec<Point>) -> Self {
        Self { vertices }
    }
}

impl From<BBox> for Polygon {
    fn from(bbox: BBox) -> Self {
        let hx = bbox.size.0 * 0.5;
        let hy = bbox.size.1 * 0.5;
        let c = bbox.angle.cos();
        let s = bbox.angle.sin();
        let local_corners = [(-hx, -hy), (-hx, hy), (hx, hy), (hx, -hy)];
        Self {
            vertices: local_corners.into_iter().map(|(x, y)| {
                Point::new((bbox.center.x + x * c - y * s, bbox.center.y + x * s + y * c))
            }).collect::<Vec<_>>().into(),
        }
    }
}

impl FloatPointCompatible for Point {
    type Scalar = f32;
    fn from_xy(x: f32, y: f32) -> Self {
        Point { x: x.into(), y: y.into() }
    }
    fn x(&self) -> f32 { self.x.into_inner() }
    fn y(&self) -> f32 { self.y.into_inner() }
}

pub struct RoomTopology {
    pub borders: Vec<Polygon>,
    pub holes: Vec<Polygon>,
}

impl RoomTopology {
    pub fn filter_polygons(&self, polygons: Vec<Polygon>) -> Vec<Polygon> {

        polygons.into_iter()
            .filter(|poly| {
                let c = poly.centroid();
                let inside_border = self.borders.iter().any(|b| b.contains(c));
                let inside_hole = self.holes.iter().any(|h| h.contains(c));
                inside_border && !inside_hole
            })
            .collect()
    }

    fn new(borders: Vec<Polygon>, holes: Vec<Polygon>) -> Self {
        Self { borders, holes }
    }

    pub fn render_rerun(&self, rec: &RecordingStream) -> Result<(), Box<dyn std::error::Error>> {
        let mut border_strips = Vec::new();
        for border in &self.borders {
            if border.vertices.is_empty() { continue; }
            let mut strip: Vec<[f32; 2]> = border.vertices.iter()
                .map(|p| [p.x.into_inner(), p.y.into_inner()])
                .collect();
            strip.push([border.vertices[0].x.into_inner(), border.vertices[0].y.into_inner()]);
            border_strips.push(strip);
        }
        rec.log("topology/borders", &LineStrips2D::new(border_strips).with_colors([Color::from_rgb(0, 200, 255)]))?;

        let mut hole_strips = Vec::new();
        for hole in &self.holes {
            if hole.vertices.is_empty() { continue; }
            let mut strip: Vec<[f32; 2]> = hole.vertices.iter()
                .map(|p| [p.x.into_inner(), p.y.into_inner()])
                .collect();
            strip.push([hole.vertices[0].x.into_inner(), hole.vertices[0].y.into_inner()]);
            hole_strips.push(strip);
        }
        rec.log("topology/holes", &LineStrips2D::new(hole_strips).with_colors([Color::from_rgb(255, 100, 100)]))?;
        Ok(())
    }

    pub fn is_segment_intersecting(&self, segment: (Point, Point)) -> bool {
        for poly in &self.borders {
            if poly.intersect(segment) {
                return true;
            }
        }
        for poly in &self.holes {
            if poly.intersect(segment) {
                return true;
            }
        }
        false
    }
}

impl From<&Data> for RoomTopology {
    fn from(data: &Data) -> Self {
        let bboxes = data.bboxes.clone();
        let poly_bboxes: Vec<Polygon> = bboxes.iter().map(|b| b.clone().into()).collect();
        let room_graph: RoomGraph = data.into();
        let room_polygons = graph_into_polygons(&room_graph);
        let doors_polygons = extract_doors_as_polygons(&room_graph);

        let borders = clip_doors(room_polygons, doors_polygons);

        let borders: Vec<Polygon> = borders.iter()
            .filter_map(shrink_polygon)
            .collect();

        let holes: Vec<Polygon> = poly_bboxes.iter()
            .filter_map(grow_polygon)
            .collect();

        let (borders, holes) = merge_polygons(borders, holes);

        RoomTopology::new(borders, holes)
    }
}


fn polygon_to_contour(polygon: &Polygon) -> Vec<Point> {
    polygon.vertices.clone()
}

fn clip_doors(borders: Vec<Polygon>, doors: Vec<Polygon>) -> Vec<Polygon> {
    let subj: Vec<Vec<Point>> = borders.iter().map(polygon_to_contour).collect();
    let clip: Vec<Vec<Point>> = doors.iter().map(polygon_to_contour).collect();
    let result = subj.overlay(&clip, OverlayRule::Union, FillRule::EvenOdd);
    result.into_iter()
        .filter_map(|shape| shape.into_iter().next())
        .map(|outer| outer.into())
        .collect()
}

fn merge_holes(holes: Vec<Polygon>) -> Vec<Polygon> {
    if holes.is_empty() { return vec![]; }

    let mut accumulated: Vec<Vec<[f32; 2]>> = vec![
        holes[0].vertices.iter()
            .map(|v| [v.x.into_inner(), v.y.into_inner()])
            .collect()
    ];
    for hole in holes.into_iter().skip(1) {
        let next: Vec<Vec<[f32; 2]>> = vec![
            hole.vertices.iter()
                .map(|v| [v.x.into_inner(), v.y.into_inner()])
                .collect()
        ];
        accumulated = accumulated.overlay(&next, OverlayRule::Union, FillRule::EvenOdd)
            .into_iter()
            .flat_map(|shape| shape.into_iter())
            .collect();
    }
    accumulated.into_iter()
        .map(|contour| contour.into_iter()
            .map(|p| Point { x: p[0].into(), y: p[1].into() })
            .collect::<Vec<_>>()
            .into()
        )
        .collect()
}

fn merge_polygons(borders: Vec<Polygon>, holes: Vec<Polygon>) -> (Vec<Polygon>, Vec<Polygon>) {
    if holes.is_empty() { return (borders, vec![]); }

    let merged_holes = merge_holes(holes);

    let borders_2d: Vec<Vec<[f32; 2]>> = borders.iter()
        .map(|p| p.vertices.iter().map(|v| [v.x.into_inner(), v.y.into_inner()]).collect())
        .collect();

    let mut independent_holes: Vec<Vec<[f32; 2]>> = vec![];
    let mut border_overlapping_holes: Vec<Vec<[f32; 2]>> = vec![];

    for hole in &merged_holes {
        let hole_2d: Vec<Vec<[f32; 2]>> = vec![
            hole.vertices.iter().map(|v| [v.x.into_inner(), v.y.into_inner()]).collect()
        ];
        let difference = hole_2d.overlay(&borders_2d, OverlayRule::Difference, FillRule::EvenOdd);
        if difference.is_empty() {
            independent_holes.push(hole.vertices.iter().map(|v| [v.x.into_inner(), v.y.into_inner()]).collect());
        } else {
            border_overlapping_holes.push(hole.vertices.iter().map(|v| [v.x.into_inner(), v.y.into_inner()]).collect());
        }
    }

    let new_borders = if border_overlapping_holes.is_empty() {
        borders
    } else {
        borders_2d.overlay(&border_overlapping_holes, OverlayRule::Difference, FillRule::EvenOdd)
            .into_iter()
            .filter_map(|shape| shape.into_iter().next())
            .map(|contour| contour.into_iter()
                .map(|p| Point { x: p[0].into(), y: p[1].into() })
                .collect::<Vec<_>>()
                .into()
            )
            .collect()
    };

    let new_holes: Vec<Polygon> = independent_holes.into_iter()
        .map(|contour| contour.into_iter()
            .map(|p| Point { x: p[0].into(), y: p[1].into() })
            .collect::<Vec<_>>()
            .into()
        )
        .collect();

    (new_borders, new_holes)
}

pub fn extract_doors_as_polygons(graph: &RoomGraph) -> Vec<Polygon> {
    if graph.edges.is_empty() { return vec![]; }
    let door_frame_offset = OrderedFloat(0.15);
    let mut polygons = Vec::new();
    for edge in &graph.edges {
        if edge.doors.is_empty() { continue; }
        let unit_dir = (edge.b - edge.a).to_unit();
        let normal = Point { x: -unit_dir.y, y: unit_dir.x };
        let offset_vec = Point { x: normal.x * door_frame_offset, y: normal.y * door_frame_offset };
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
    if graph.edges.is_empty() { return vec![]; }
    let mut remaining: Vec<Edge> = graph.edges.clone();
    let mut polygons: Vec<Polygon> = Vec::new();
    while !remaining.is_empty() {
        let mut ordered: Vec<Edge> = Vec::new();
        ordered.push(remaining.remove(0));
        loop {
            let last_point = ordered.last().unwrap().b;
            let last_snapped = last_point.snap();
            if let Some(pos) = remaining.iter().position(|w| w.a == last_point || w.a.snap() == last_snapped) {
                ordered.push(remaining.remove(pos));
            } else if let Some(pos) = remaining.iter().position(|w| w.b == last_point || w.b.snap() == last_snapped) {
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

use i_overlay::mesh::outline::offset::OutlineOffset;
use i_overlay::mesh::style::{LineJoin, OutlineStyle};

const WALL_MARGIN: f32 = 0.1;    // shrink borders inward
const OBJECT_MARGIN: f32 = 0.1;  // grow holes outward

fn shrink_polygon(polygon: &Polygon) -> Option<Polygon> {
    offset_polygon(polygon, -WALL_MARGIN)
}

fn grow_polygon(polygon: &Polygon) -> Option<Polygon> {
    offset_polygon(polygon, OBJECT_MARGIN)
}

fn offset_polygon(polygon: &Polygon, amount: f32) -> Option<Polygon> {
    let mut contour: Vec<[f32; 2]> = polygon.vertices.iter()
        .map(|p| [p.x.into_inner(), p.y.into_inner()])
        .collect();

    if signed_area(&contour) < 0.0 {
        contour.reverse();
    }

    let style = OutlineStyle::new(amount).line_join(LineJoin::Miter(2.0));
    vec![contour].outline(&style)
        .into_iter()
        .next()
        .and_then(|s| s.into_iter().next())
        .map(|contour| {
            contour.into_iter()
                .map(|p| Point { x: p[0].into(), y: p[1].into() })
                .collect::<Vec<_>>()
                .into()
        })
}

fn signed_area(contour: &[[f32; 2]]) -> f32 {
    let n = contour.len();
    let mut area = 0.0f32;
    for i in 0..n {
        let a = contour[i];
        let b = contour[(i + 1) % n];
        area += a[0] * b[1] - b[0] * a[1];
    }
    area / 2.0
}
