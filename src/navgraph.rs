use ordered_float::OrderedFloat;
use std::collections::BinaryHeap;
use std::cmp::Reverse;
use rerun::{Points2D, RecordingStream, LineStrips2D, Color};

use crate::utils::Point;
use crate::navmesh::NavMesh;

pub struct NavNode {
    pub centroid: Point,
    pub polygon_index: usize,
}

pub struct NavEdge {
    pub to: usize,
    pub cost: OrderedFloat<f32>,
}

pub struct NavGraph {
    pub nodes: Vec<NavNode>,
    pub edges: Vec<Vec<NavEdge>>,
}

impl From<&NavMesh> for NavGraph {
    fn from(mesh: &NavMesh) -> Self {
        let nodes: Vec<NavNode> = mesh.polygons.iter().enumerate().map(|(i, poly)| {
            NavNode {
                centroid: poly.centroid(),
                polygon_index: i,
            }
        }).collect();

        let edges: Vec<Vec<NavEdge>> = mesh.adjacency.iter().enumerate().map(|(i, neighbours)| {
            neighbours.iter().map(|&j| {
                let a = nodes[i].centroid;
                let b = nodes[j].centroid;
                let cost = (b - a).length();
                NavEdge { to: j, cost }
            }).collect()
        }).collect();

        NavGraph { nodes, edges }
    }
}

impl NavGraph {
    pub fn render_rerun(
        &self,
        rec: &RecordingStream,
    ) -> Result<(), Box<dyn std::error::Error>> {

        // -------------------------
        // NODES (centroids)
        // -------------------------
        let points: Vec<[f32; 2]> = self
            .nodes
            .iter()
            .map(|n| [
                n.centroid.x.into_inner(),
                n.centroid.y.into_inner(),
            ])
            .collect();

        rec.log(
            "navgraph/nodes",
            &Points2D::new(points),
        )?;

        // -------------------------
        // EDGES (graph connections)
        // -------------------------
        let mut edge_lines = Vec::new();

        for (i, edges) in self.edges.iter().enumerate() {
            let a = self.nodes[i].centroid;

            for edge in edges {
                let b = self.nodes[edge.to].centroid;

                edge_lines.push(vec![
                    [a.x.into_inner(), a.y.into_inner()],
                    [b.x.into_inner(), b.y.into_inner()],
                ]);
            }
        }

        rec.log(
            "navgraph/edges",
            &LineStrips2D::new(edge_lines)
                .with_colors([Color::from_rgb(80, 80, 255)]), // blue-ish
        )?;

        Ok(())
    }

    pub fn astar(&self, start: usize, goal: usize) -> Option<Vec<usize>> {
        let n = self.nodes.len();
        let mut g_score = vec![f32::INFINITY; n];
        let mut prev: Vec<Option<usize>> = vec![None; n];
        let mut heap = BinaryHeap::new();

        g_score[start] = 0.0;
        heap.push(Reverse((OrderedFloat(0.0f32), start)));

        while let Some(Reverse((f, u))) = heap.pop() {
            if u == goal {
                let mut path = vec![];
                let mut cur = goal;
                loop {
                    path.push(cur);
                    match prev[cur] {
                        None => break,
                        Some(p) => cur = p,
                    }
                }
                path.reverse();
                return Some(path);
            }

            // stale check against f_score: recompute expected f for u
            let h_u = (self.nodes[goal].centroid - self.nodes[u].centroid).length().into_inner();
            if f.into_inner() > g_score[u] + h_u + 1e-4 { continue; }

            for edge in &self.edges[u] {
                let tentative_g = g_score[u] + edge.cost.into_inner();
                if tentative_g < g_score[edge.to] {
                    g_score[edge.to] = tentative_g;
                    prev[edge.to] = Some(u);
                    let h = (self.nodes[goal].centroid - self.nodes[edge.to].centroid)
                        .length()
                        .into_inner();
                    heap.push(Reverse((OrderedFloat(tentative_g + h), edge.to)));
                }
            }
        }
        None
    }

    pub fn render_rerun_path(
        &self,
        path: &[usize],
        rec: &RecordingStream,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Waypoint centroids as points
        let points: Vec<[f32; 2]> = path.iter().map(|&i| {
            let c = self.nodes[i].centroid;
            [c.x.into_inner(), c.y.into_inner()]
        }).collect();

        //rec.log("nav/waypoints", &Points2D::new(points.clone()))?;

        // Connect them as a line strip
        if path.len() >= 2 {
            rec.log("nav/path", &LineStrips2D::new(vec![points]))?;
        }

        Ok(())
    }
}
