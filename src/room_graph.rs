use crate::utils::Point;

struct Node {
    id: usize,
    pos: Point,
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
