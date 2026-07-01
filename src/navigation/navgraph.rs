use std::{fs, process};
use std::io::prelude::*;
use ordered_float::OrderedFloat;
use std::collections::BinaryHeap;
use std::cmp::Reverse;
use rerun::{Points2D, RecordingStream, LineStrips2D, Color};

use super::utils::Point;
use super::parser::parse_raw_data;
use super::{data::Data, room_topology::RoomTopology, navmesh::NavMesh};

#[derive(Debug)]
pub struct NavNode {
    pub centroid: Point,
    pub polygon_index: usize,
}

#[derive(Debug)]
pub struct NavEdge {
    pub to: usize,
    pub cost: OrderedFloat<f32>,
}

#[derive(Debug)]
pub struct NavGraph {
    pub nodes: Vec<NavNode>,
    pub edges: Vec<Vec<NavEdge>>,
    room_data: Data,
    room_topology: RoomTopology,
    navmesh: NavMesh,
}

impl NavGraph {
    pub fn new(filepath: &str) -> Self {
        let file_name = "/ase_scene_language.txt";
        let mut file = fs::File::open(String::from(filepath) + file_name)
            .unwrap_or_else( |e| { eprintln!("{}: '{}'", e, String::from(filepath) + file_name); process::exit(1) });
        let mut contents = String::new();
        let _ = file.read_to_string(&mut contents);

        let room_raw_data = parse_raw_data(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
        let room_data: Data = room_raw_data.into();

        let room_topology: RoomTopology = (&room_data).into();
        let navmesh: NavMesh = (&room_topology).into();

        let nodes: Vec<NavNode> = navmesh.polygons.iter().enumerate().map(|(i, poly)| {
            NavNode {
                centroid: poly.centroid(),
                polygon_index: i,
            }
        }).collect();

        let edges: Vec<Vec<NavEdge>> = navmesh.adjacency.iter().enumerate().map(|(i, neighbours)| {
            neighbours.iter().map(|&j| {
                let a = nodes[i].centroid;
                let b = nodes[j].centroid;
                let cost = (b - a).length();
                NavEdge { to: j, cost }
            }).collect()
        }).collect();

        NavGraph { nodes, edges, room_data, room_topology, navmesh }
    }

    pub fn log(
        &self,
        rec: &RecordingStream,
        log_path: &str,
    ) -> anyhow::Result<()> {
        let _ = self.room_data.log(&rec, log_path);
        let _ = self.room_topology.log(&rec, log_path);
        let _ = self.navmesh.log(&rec, log_path);

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
            format!("{}/navgraph/nodes", log_path).as_str(),
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
            format!("{}/navgraph/edges", log_path).as_str(),
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
                return Some(self.path_straightening(path));
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
        log_path: &str,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Waypoint centroids as points
        let points: Vec<[f32; 2]> = path.iter().map(|&i| {
            let c = self.nodes[i].centroid;
            [c.x.into_inner(), c.y.into_inner()]
        }).collect();

        //rec.log("nav/waypoints", &Points2D::new(points.clone()))?;

        // Connect them as a line strip
        if path.len() >= 2 {
            rec.log(String::from(log_path) + "nav/path", &LineStrips2D::new(vec![points]))?;
        }

        Ok(())
    }

    fn path_straightening(&self, path: Vec<usize>) -> Vec<usize> {
        if path.len() <= 2 { return path; }

        let n = path.len();
        let mut result = vec![path[0]];
        let mut current = 0;

        while current < n - 1 {
            // find the furthest node we can reach from current without obstruction
            let mut furthest = current + 1;
            for next in (current + 2)..n {
                let a = self.nodes[path[current]].centroid;
                let b = self.nodes[path[next]].centroid;
                if !self.room_topology.is_segment_intersecting((a, b)) {
                    furthest = next;
                } else {
                    break;
                }
            }
            result.push(path[furthest]);
            current = furthest;
        }

        result
    }
}
