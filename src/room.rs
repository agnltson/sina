pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use crate::room::wall::Wall;
use crate::room::door::Door;
use crate::room::bbox::Bbox;

#[derive(Debug)]
pub struct Room {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<Bbox>,
}

impl Room {
    pub fn new(walls: Vec<Wall>, doors: Vec<Door>, bboxes: Vec<Bbox>) -> Self {
        Self {
            walls,
            doors,
            bboxes,
        }
    }
}

use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

impl Into<ConstrainedDelaunayTriangulation<Point2<f64>>> for Room {
    fn into(self) -> ConstrainedDelaunayTriangulation<Point2<f64>> {
        let mut cdt = ConstrainedDelaunayTriangulation::new();

        let walls = Vec::new();
        for (from, to) in walls.into_iter() {
            let _ = cdt.add_constraint_edge(from, to);
        }

        let obstacles: Vec<Vec<_>> = Vec::new();
        for obstacle_bound in obstacles.into_iter() {
            let _ = cdt.add_constraint_edges(obstacle_bound, true);
        }

        cdt
    }
}
