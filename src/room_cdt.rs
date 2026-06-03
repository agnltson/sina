use std::collections::HashMap;
use ordered_float::OrderedFloat;

use crate::room_graph::{RoomGraph, Edge};
use crate::utils::Point;
use crate::data::bbox::BBox;

struct RoomCDT {
    constrained: Vec<(Point, Point)>,
}

impl From<RoomGraph> for RoomCDT {
    fn from(room_graph: RoomGraph) -> Self {
        Self {
            constrained: Vec::new(),
        }
    }
}

fn same_boundary(id_to_points: HashMap<usize, Point>, w1: &Edge, w2: &Edge) -> bool {
    const CLOSENESS_THRESHOLD: OrderedFloat<f32> = OrderedFloat(0.25);
    const PARALLELNESS_THRESHOLD: OrderedFloat<f32> = OrderedFloat(1.0);

    let dir1 = w1.b - w1.a;
    let dir2 = w2.b - w2.a;

    let len1 = dir1.length();
    let len2 = dir2.length();

    if len1 < OrderedFloat(f32::EPSILON) || len2 < OrderedFloat(f32::EPSILON) {
        return false;
    }

    let unit1 = dir1.to_unit();
    let unit2 = dir2.to_unit();

    let parallel = OrderedFloat(unit1.dot(unit2).abs()) < PARALLELNESS_THRESHOLD;

    if !parallel {
        return false;
    }

    let offset = OrderedFloat((w1.a.x - w2.a.x).abs() + (w1.a.y - w2.a.y).abs());

    offset < CLOSENESS_THRESHOLD
}
