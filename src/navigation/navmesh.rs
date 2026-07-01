use std::collections::HashMap;
use spade::Triangulation;
use super::room_cdt::RoomCDT;
use super::room_topology::RoomTopology;
use super::utils::{Point, Polygon};

#[derive(Debug)]
pub struct NavPolygon {
    pub vertices: Vec<Point>,
    pub neighbours: Vec<usize>,
}

#[derive(Debug)]
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
    pub fn log(
        &self,
        rec: &RecordingStream,
        log_path: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // --- Polygon vertices ---
        let mut all_vertices: Vec<[f32; 2]> = Vec::new();

        for poly in &self.polygons {
            for v in &poly.vertices {
                all_vertices.push([v.x.into_inner(), v.y.into_inner()]);
            }
        }

        rec.log(
            format!("{}/navmesh/vertices", log_path).as_str(),
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
            format!("{}/navmesh/polygons", log_path).as_str(),
            &LineStrips2D::new(edges),
        )?;

        Ok(())
    }
}
