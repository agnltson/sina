use ordered_float::OrderedFloat;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use super::room_topology::RoomTopology;
use super::utils::{Point, Polygon};

pub struct RoomCDT {
    pub cdt: ConstrainedDelaunayTriangulation<Point2<f32>>,
}

fn insert_polygon(cdt: &mut ConstrainedDelaunayTriangulation<Point2<f32>>, polygon: &Polygon) {
    let len = polygon.vertices.len();
    if len < 2 { return; }
    for i in 0..len {
        let a: Point2<f32> = polygon.vertices[i].into();
        let b: Point2<f32> = polygon.vertices[(i + 1) % len].into();
        let (Ok(va), Ok(vb)) = (cdt.insert(a), cdt.insert(b)) else {
            continue;
        };
        if cdt.can_add_constraint(va, vb) {
            cdt.add_constraint(va, vb);
        }
    }
}

impl From<&RoomTopology> for RoomCDT {
    fn from(room_topo: &RoomTopology) -> Self {
        let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f32>>::new();
        for polygon in &room_topo.borders {
            insert_polygon(&mut cdt, polygon);
        }
        for polygon in &room_topo.holes {
            insert_polygon(&mut cdt, polygon);
        }
        Self { cdt }
    }
}

impl From<&RoomCDT> for Vec<Polygon> {
    fn from(room_cdt: &RoomCDT) -> Self {
        room_cdt.cdt.inner_faces().map(|face| {
            let vertices = face.vertices().map(|v| {
                let p = v.position();
                Point {
                    x: OrderedFloat(p.x),
                    y: OrderedFloat(p.y),
                }
            }).to_vec();
            Polygon::new(vertices)
        }).collect()
    }
}
