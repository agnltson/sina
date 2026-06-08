use std::collections::HashMap;
use spade::Triangulation;
use ordered_float::OrderedFloat;
use crate::room_cdt::RoomCDT;
use crate::room_topology::RoomTopology;
use crate::utils::{Point, Polygon};

pub struct NavPolygon {
    pub vertices: Vec<Point>,
    pub neighbours: Vec<usize>,
}

pub struct NavMesh {
    pub polygons: Vec<Polygon>,
    pub adjacency: Vec<Vec<usize>>,
}

impl From<&RoomTopology> for NavMesh {
    fn from(room_topology: &RoomTopology) -> Self {
    let room_cdt: RoomCDT = room_topology.into();
    let cdt_polygons: Vec<Polygon> = (&room_cdt).into();

    let polygons: Vec<Polygon> = room_topology.filter_polygons(cdt_polygons);

    let n = polygons.len();
    // Map each edge (sorted pair of points) -> list of triangle indices that own it
    let mut edge_map: HashMap<(Point, Point), Vec<usize>> = HashMap::new();
    for (i, poly) in polygons.iter().enumerate() {
        let verts = &poly.vertices;
        for j in 0..verts.len() {
            let a = verts[j];
            let b = verts[(j + 1) % verts.len()];
            // Sort so (a,b) and (b,a) map to the same key
            let key = if a < b { (a, b) } else { (b, a) };
            edge_map.entry(key).or_default().push(i);
        }
    }
    let mut adjacency = vec![vec![]; n];
    for (_, owners) in &edge_map {
        if owners.len() == 2 {
            adjacency[owners[0]].push(owners[1]);
            adjacency[owners[1]].push(owners[0]);
        }
    }
    Self { polygons, adjacency }
    }
}

use rerun::{
    RecordingStream,
    Points2D,
    LineStrips2D,
};

impl NavMesh {
    pub fn render_rerun(
        &self,
        rec: &RecordingStream,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // --- Polygon vertices ---
        let mut all_vertices: Vec<[f32; 2]> = Vec::new();

        for poly in &self.polygons {
            for v in &poly.vertices {
                all_vertices.push([v.x.into_inner(), v.y.into_inner()]);
            }
        }

        rec.log(
            "navmesh/vertices",
            &Points2D::new(all_vertices),
        )?;

        // --- Polygon edges ---
        let mut edges: Vec<Vec<[f32; 2]>> = Vec::new();

        for poly in &self.polygons {
            let n = poly.vertices.len();
            if n < 2 {
                continue;
            }

            for i in 0..n {
                let a = &poly.vertices[i];
                let b = &poly.vertices[(i + 1) % n];

                edges.push(vec![
                    [a.x.into_inner(), a.y.into_inner()],
                    [b.x.into_inner(), b.y.into_inner()],
                ]);
            }
        }

        rec.log(
            "navmesh/polygons",
            &LineStrips2D::new(edges),
        )?;

        Ok(())
    }
}

/*impl From<&RoomCDT> for NavMesh {
    fn from(room_cdt: &RoomCDT) -> Self {
        let cdt = &room_cdt.cdt;

        // Assign a stable index to every inner face
        let face_index: HashMap<_, usize> = cdt
            .inner_faces()
            .enumerate()
            .map(|(i, f)| (f.fix(), i))
            .collect();

        let polygons = cdt.inner_faces().map(|face| {
            // The three vertices of this triangle
            let vertices: Vec<Point> = face
                .vertices()
                .iter()
                .map(|v| {
                    let p = v.position();
                    Point {
                        x: OrderedFloat(p.x),
                        y: OrderedFloat(p.y),
                    }
                })
                .collect();

            // Adjacent inner faces become neighbours (outer/infinite face is skipped)
            let neighbours: Vec<usize> = face
                .adjacent_edges()
                .iter()
                .filter_map(|edge| {
                    let twin_face = edge.rev().face();
                    twin_face
                        .as_inner()
                        .and_then(|inner| face_index.get(&inner.fix()).copied())
                })
                .collect();

            NavPolygon { vertices, neighbours }
        }).collect();

        NavMesh { polygons }
    }
}*/
