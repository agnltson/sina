use crate::utils::Point;

struct Node {
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

struct Edge {
    id: usize,
    a: usize, // Node id
    b: usize, // Node id
}

impl Edge {
    pub fn new(id: usize, a: usize, b: usize) -> Self {
        Self {
            id,
            a,
            b,
        }
    }
}

pub struct RoomGraph {
    nodes: Vec<Node>,
    edges: Vec<Edge>,
}

impl RoomGraph {
    pub fn new(nodes: Vec<Node>, edges: Vec<Edge>) -> Self {
        Self {
            nodes,
            edges,
        }
    }
}

use std::collections::HashMap;
use crate::data::Data;

impl From<Data> for RoomGraph {
    fn from(data: Data) -> Self {
        let mut node_id = 0;
        let mut edge_id = 0;

        let mut nodes: Vec<Node> = Vec::new();
        let mut edges: Vec<Edge> = Vec::new();
        let mut id_to_point = HashMap::new();
        let mut point_to_id = HashMap::new();

        let walls = data.walls;

        for (start, end) in walls.iter().map(|w| (w.a, w.b)){

            // No dup points
            if !point_to_id.contains_key(&start) {
                nodes.push(Node::new(node_id, start));
                id_to_point.insert(node_id, start);
                point_to_id.insert(start, node_id);
                node_id += 1;
            }
            if !point_to_id.contains_key(&end) {
                nodes.push(Node::new(node_id, end));
                id_to_point.insert(node_id, end);
                point_to_id.insert(end, node_id);
                node_id += 1;
            }

            // Set edges without dup
            let (start_node_id, end_node_id) = (point_to_id.get(&start).unwrap(), point_to_id.get(&end).unwrap());

            let edges_id: Vec<_> = edges.iter().map(|e| (e.a, e.b)).collect();
            if !(edges_id.contains(&(*start_node_id, *end_node_id)) || edges_id.contains(&(*end_node_id, *start_node_id))) {
                edges.push(Edge::new(edge_id, *start_node_id, *end_node_id));
                edge_id += 1;
            }
        }

        RoomGraph::new(nodes, edges)
    }
}
