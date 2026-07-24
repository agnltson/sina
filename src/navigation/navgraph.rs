use std::{fs, process};
use std::io::prelude::*;
use ordered_float::OrderedFloat;
use std::collections::BinaryHeap;
use std::cmp::Reverse;

use super::utils::Point;
use super::parser::parse_raw_data;
use super::{data::Data, room_topology::RoomTopology, navmesh::NavMesh};

#[derive(Debug)]
pub struct NavNode {
    pub centroid: Point,
}

#[derive(Debug)]
pub struct NavEdge {
    pub to: usize,
    pub cost: f32,
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
        let file_name = "/semantic.txt";
        let mut file = fs::File::open(String::from(filepath) + file_name)
            .unwrap_or_else( |e| { eprintln!("{}: '{}'", e, String::from(filepath) + file_name); process::exit(1) });
        let mut contents = String::new();
        let _ = file.read_to_string(&mut contents);

        let room_raw_data = parse_raw_data(&mut contents.trim()).unwrap_or_else( |e| { eprintln!("{}", e); process::exit(1) });
        let room_data: Data = room_raw_data.into();

        let room_topology: RoomTopology = (&room_data).into();
        let navmesh: NavMesh = (&room_topology).into();

        let nodes: Vec<NavNode> = navmesh.polygons.iter().map(|poly| {
            NavNode {
                centroid: poly.centroid(),
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

    pub fn find_path(&self, start_point: Point, goal_point: Point) -> Option<Vec<Point>> {
        let start_idx = self.nearest_centroid(start_point)?;
        let goal_idx = self.nearest_centroid(goal_point)?;
        let raw_path = self.astar_raw(start_idx, goal_idx)?;
        Some(self.funnel(&raw_path, start_point, goal_point))
    }

    pub fn astar_raw(&self, start: usize, goal: usize) -> Option<Vec<usize>> {
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
            let h_u = (self.nodes[goal].centroid - self.nodes[u].centroid).length();
            if f.into_inner() > g_score[u] + h_u + 1e-4 { continue; }

            for edge in &self.edges[u] {
                let tentative_g = g_score[u] + edge.cost;
                if tentative_g < g_score[edge.to] {
                    g_score[edge.to] = tentative_g;
                    prev[edge.to] = Some(u);
                    let h = (self.nodes[goal].centroid - self.nodes[edge.to].centroid).length();
                    heap.push(Reverse((OrderedFloat(tentative_g + h), edge.to)));
                }
            }
        }
        None
    }

    fn shared_edge(&self, from_idx: usize, to_idx: usize) -> Option<(Point, Point)> {
        let from_poly = &self.navmesh.polygons[from_idx];
        let to_poly = &self.navmesh.polygons[to_idx];
        let n = from_poly.vertices.len();

        for i in 0..n {
            let a = from_poly.vertices[i];
            let b = from_poly.vertices[(i + 1) % n];

            let a_shared = to_poly.vertices.iter().any(|&v| v == a);
            let b_shared = to_poly.vertices.iter().any(|&v| v == b);

            if a_shared && b_shared {
                return Some((a, b));
            }
        }
        None
    }

    pub fn funnel(&self, path: &[usize], start: Point, goal: Point) -> Vec<Point> {
        if path.len() < 2 {
            return vec![start, goal];
        }

        let mut portals: Vec<(Point, Point)> = Vec::with_capacity(path.len() + 1);
        portals.push((start, start));

        for w in path.windows(2) {
            let (from, to) = (w[0], w[1]);
            match self.shared_edge(from, to) {
                Some((right, left)) => portals.push((left, right)),
                None => {
                    let c = self.nodes[to].centroid;
                    portals.push((c, c));
                }
            }
        }
        portals.push((goal, goal));

        let mut result = vec![start];
        let mut apex = start;
        let mut left = portals[0].0;
        let mut right = portals[0].1;
        let (mut left_idx, mut right_idx) = (0usize, 0usize);

        let mut i = 1;
        while i < portals.len() {
            let (pl, pr) = portals[i];

            if triarea2(apex, right, pr) <= 0.0 {
                if apex == right || triarea2(apex, left, pr) > 0.0 {
                    right = pr;
                    right_idx = i;
                } else {
                    result.push(left);
                    apex = left;
                    left = apex;
                    right = apex;
                    right_idx = left_idx;
                    i = left_idx + 1;
                    continue;
                }
            }

            if triarea2(apex, left, pl) >= 0.0 {
                if apex == left || triarea2(apex, right, pl) < 0.0 {
                    left = pl;
                    left_idx = i;
                } else {
                    result.push(right);
                    apex = right;
                    left = apex;
                    right = apex;
                    left_idx = right_idx;
                    i = right_idx + 1;
                    continue;
                }
            }

            i += 1;
        }

        result.push(goal);
        result
    }

    pub fn get_node_positions(&self) -> Vec<Point> {
        self.nodes
            .iter()
            .map(|node| node.centroid)
            .collect()
    }

    pub fn nearest_centroid(&self, point: Point) -> Option<usize> {
        let in_room = self.navmesh.polygons.iter().any(|poly| poly.contains(point));
        if !in_room {
            return None;
        }

        self.nodes
            .iter()
            .enumerate()
            .min_by_key(|(_, node)| OrderedFloat((node.centroid - point).length()))
            .map(|(i, _)| i)
    }

    pub fn polygon_vertices(&self) -> Vec<Vec<Point>> {
        self.navmesh.polygons.iter()
            .map(|p| p.vertices.clone())
            .collect()
    }

    pub fn edges_as_points(&self) -> Vec<(Point, Point)> {
        self.edges.iter().enumerate()
            .flat_map(|(i, edges)| {
                let a = self.nodes[i].centroid;
                edges.iter().map(move |e| (a, self.nodes[e.to].centroid))
            })
            .collect()
    }
}

fn triarea2(a: Point, b: Point, c: Point) -> f32 {
    let ax = b.x.into_inner() - a.x.into_inner();
    let ay = b.y.into_inner() - a.y.into_inner();
    let bx = c.x.into_inner() - a.x.into_inner();
    let by = c.y.into_inner() - a.y.into_inner();
    bx * ay - ax * by
}
