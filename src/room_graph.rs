use std::collections::HashMap;
use rerun::{LineStrips2D, Points2D, RecordingStream, Color};
use ordered_float::OrderedFloat;

use crate::utils::Point;
use crate::data::{Data, door::Door, bbox::BBox};
use crate::raw_data::RawData;

pub struct Node {
    pub id: usize,
    pub pos: Point,
}

impl Node {
    pub fn new(id: usize, pos: Point) -> Self {
        Self {
            id,
            pos,
        }
    }
}

pub struct Edge {
    pub id: usize,
    pub a: Point,
    pub b: Point,
    pub doors: Vec<Door>,
}

impl Edge {
    pub fn new(id: usize, a: Point, b: Point, doors: Vec<Door>) -> Self {
        Self {
            id,
            a,
            b,
            doors,
        }
    }
}

pub struct RoomGraph {
    pub nodes: HashMap<usize, Point>,
    pub edges: Vec<Edge>,
}

impl RoomGraph {
    pub fn new(nodes: HashMap<usize, Point>, edges: Vec<Edge>) -> Self {
        Self {
            nodes,
            edges,
        }
    }
}

impl From<Data> for RoomGraph {
    fn from(data: Data) -> Self {
        let mut node_id = 0;
        let mut edge_id = 0;

        let mut edges: Vec<Edge> = Vec::new();
        let mut id_to_point = HashMap::new();
        let mut point_to_id = HashMap::new();

        let walls = data.walls;

        for (id, start, end) in walls.iter().map(|w| (w.id, w.a, w.b)){

            // No dup points
            if !point_to_id.contains_key(&start) {
                id_to_point.insert(node_id, start);
                point_to_id.insert(start, node_id);
                node_id += 1;
            }
            if !point_to_id.contains_key(&end) {
                id_to_point.insert(node_id, end);
                point_to_id.insert(end, node_id);
                node_id += 1;
            }

            // Set edges without dup
            let (start_node_id, end_node_id) = (point_to_id.get(&start).unwrap(), point_to_id.get(&end).unwrap());

            let edges_id: Vec<_> =
                edges.iter()
                .map(|e| (e.a, e.b))
                .collect();
            if !(edges_id.contains(&(start, end)) || edges_id.contains(&(end, start))) {

                // extract door on that wall
                let attached_doors: Vec<Door> =
                    data.doors.clone()
                    .into_iter()
                    .filter(|d| d.wall_id == id)
                    .collect();

                edges.push(Edge::new(edge_id, start, end, attached_doors));
                edge_id += 1;
            }
        }

        RoomGraph::new(id_to_point, edges)
    }
}

impl From<RawData> for RoomGraph {
    fn from(raw_data: RawData) -> Self {
        let data: Data = raw_data.into();
        data.into()
    }
}

impl RoomGraph {
    pub fn render_rerun(
        &self,
        rec: &RecordingStream,
    ) -> Result<(), Box<dyn std::error::Error>> {

        // -------------------------
        // NODES
        // -------------------------
        let mut points = Vec::new();

        for (_id, pos) in &self.nodes {
            points.push([
                pos.x.into_inner(),
                pos.y.into_inner(),
            ]);
        }

        rec.log(
            "graph/nodes",
            &Points2D::new(points),
        )?;

        // -------------------------
        // EDGES
        // -------------------------
        let mut lines = Vec::new();

        for edge in &self.edges {
            let a = edge.a;
            let b = edge.b;

            lines.push(vec![
                [a.x.into_inner(), a.y.into_inner()],
                [b.x.into_inner(), b.y.into_inner()],
            ]);
        }

        rec.log(
            "graph/edges",
            &LineStrips2D::new(lines),
        )?;

        Ok(())
    }
}
