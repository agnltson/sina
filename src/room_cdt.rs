use std::collections::HashMap;
use ordered_float::OrderedFloat;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use crate::room_topology::{RoomTopology, Polygon};
use crate::utils::Point;

pub struct RoomCDT {
    cdt: ConstrainedDelaunayTriangulation<Point2<f32>>,
}

fn insert_polygon(cdt: &mut ConstrainedDelaunayTriangulation<Point2<f32>>, polygon: &Polygon) {
    let len = polygon.vertices.len();
    if len < 2 { return; }
    for i in 0..len {
        let a: Point2<f32> = polygon.vertices[i].into();
        let b: Point2<f32> = polygon.vertices[(i + 1) % len].into();
        let (Ok(va), Ok(vb)) = (cdt.insert(a), cdt.insert(b)) else {
            continue;
        };
        if cdt.can_add_constraint(va, vb) {
            cdt.add_constraint(va, vb);
        }
    }
}

impl From<RoomTopology> for RoomCDT {
    fn from(room_topo: RoomTopology) -> Self {
        let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f32>>::new();
        for polygon in &room_topo.borders {
            insert_polygon(&mut cdt, polygon);
        }
        for polygon in &room_topo.holes {
            insert_polygon(&mut cdt, polygon);
        }
        Self { cdt }
    }
}

use rerun::{LineStrips2D, Points2D, RecordingStream};

impl RoomCDT {
    pub fn render_rerun(
        &self,
        rec: &RecordingStream,
    ) -> Result<(), Box<dyn std::error::Error>> {

        let vertices: Vec<[f32; 2]> = self
            .cdt
            .vertices()
            .map(|v| {
                let p = v.position();
                [p.x, p.y]
            })
            .collect();

        rec.log(
            "cdt/vertices",
            &Points2D::new(vertices),
        )?;

        let mut tri_edges = Vec::new();

        for edge in self.cdt.undirected_edges() {
            let verts = edge.vertices();

            let a = verts[0].position();
            let b = verts[1].position();

            tri_edges.push(vec![
                [a.x, a.y],
                [b.x, b.y],
            ]);
        }

        rec.log(
            "cdt/triangulation",
            &LineStrips2D::new(tri_edges),
        )?;

        let mut constraints = Vec::new();

        for edge in self.cdt.undirected_edges() {
            if edge.is_constraint_edge() {
                let verts = edge.vertices();

                let a = verts[0].position();
                let b = verts[1].position();

                constraints.push(vec![
                    [a.x, a.y],
                    [b.x, b.y],
                ]);
            }
        }

        rec.log(
            "cdt/constraints",
            &LineStrips2D::new(constraints),
        )?;

        Ok(())
    }
}
