use std::collections::HashMap;
use ordered_float::OrderedFloat;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::room_graph::{RoomGraph, Edge};
use crate::utils::Point;
use crate::data::bbox::BBox;

pub struct RoomCDT {
    cdt: ConstrainedDelaunayTriangulation<Point2<f32>>,
}

impl From<RoomGraph> for RoomCDT {
    fn from(room_graph: RoomGraph) -> Self {
        let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f32>>::new();

        for edge in room_graph.edges.iter() {
            let _ = cdt.add_constraint_edge(
                edge.a.into(),
                edge.b.into(),
            );
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
