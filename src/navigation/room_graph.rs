use std::collections::HashMap;

use super::utils::Point;
use super::data::{Data, door::Door};
use super::raw_data::RawData;

#[derive(Debug, Clone)]
pub struct Edge {
    pub a: Point,
    pub b: Point,
    pub doors: Vec<Door>,
}

impl Edge {
    pub fn new(a: Point, b: Point, doors: Vec<Door>) -> Self {
        Self {
            a,
            b,
            doors,
        }
    }
}

pub struct RoomGraph {
    pub nodes: HashMap<i64, Point>,
    pub edges: Vec<Edge>,
}

impl RoomGraph {
    pub fn new(nodes: HashMap<i64, Point>, edges: Vec<Edge>) -> Self {
        Self {
            nodes,
            edges,
        }
    }
}

impl From<&Data> for RoomGraph {
    fn from(data: &Data) -> Self {
        let mut node_id = 0;

        let mut edges: Vec<Edge> = Vec::new();
        let mut id_to_point = HashMap::new();
        let mut point_to_id = HashMap::new();

        let walls = data.walls.clone();

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
            let (_start_node_id, _end_node_id) = (point_to_id.get(&start).unwrap(), point_to_id.get(&end).unwrap());

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

                edges.push(Edge::new(start, end, attached_doors));
            }
        }

        RoomGraph::new(id_to_point, edges)
    }
}

impl From<RawData> for RoomGraph {
    fn from(raw_data: RawData) -> Self {
        let data: Data = raw_data.into();
        (&data).into()
    }
}
