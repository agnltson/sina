pub(crate) mod wall;
pub(crate) mod door;
pub(crate) mod bbox;

use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::room::wall::Wall;
use crate::room::door::Door;
use crate::room::bbox::Bbox;
use crate::utils::{point2_add, point2_sub};

#[derive(Debug, Clone)]
pub struct Room {
    pub walls: Vec<Wall>,
    pub doors: Vec<Door>,
    pub bboxes: Vec<Bbox>,
    pub doors_proj: Vec<(Point2<f32>, Point2<f32>)>,
    pub walls_proj: Vec<(Point2<f32>, Point2<f32>)>,
    pub bboxes_proj: Vec<[Point2<f32>; 4]>,
}

impl Room {
    pub fn new(walls: Vec<Wall>, doors: Vec<Door>, bboxes: Vec<Bbox>) -> Self {
        let proj = walls_doors_ground_projection(&walls, &doors);

        Self {
            walls: walls.clone(),
            doors: doors.clone(),
            bboxes: bboxes.clone(),
            doors_proj: proj[0].clone(),
            walls_proj: proj[1].clone(),
            bboxes_proj: obstacles_ground_projection(&bboxes),
        }
    }

    pub fn render(&self) {
        use macroquad::prelude::*;

        let scale: f32 = 75.0;
        let x_offset: f32 = 400.0;
        let y_offset: f32 = 300.0;

        // Walls
        /*for (wall_start, wall_end) in &self.walls_proj {
            draw_line(
                wall_start.x * scale + x_offset,
                wall_start.y * scale + y_offset,
                wall_end.x * scale + x_offset,
                wall_end.y * scale + y_offset,
                2.0,
                BLUE,
            );
        }*/

        // Doors
        for (door_start, door_end) in &self.doors_proj {
            draw_line(
                door_start.x * scale + x_offset,
                door_start.y * scale + y_offset,
                door_end.x * scale + x_offset,
                door_end.y * scale + y_offset,
                2.0,
                GREEN,
            );
        }

        // Bounding boxes
        for bbox in &self.bboxes_proj {
            for i in 0..4 {
                let a = bbox[i];
                let b = bbox[(i + 1) % 4];

                draw_line(
                    a.x * scale + x_offset,
                    a.y * scale + y_offset,
                    b.x * scale + x_offset,
                    b.y * scale + y_offset,
                    2.0,
                    RED,
                );
            }
        }
    }
}

impl Into<ConstrainedDelaunayTriangulation<Point2<f32>>> for Room {
    fn into(self) -> ConstrainedDelaunayTriangulation<Point2<f32>> {
        let mut cdt = ConstrainedDelaunayTriangulation::new();

        let doors = self.doors_proj;
        for (from, to) in doors.into_iter() {
            println!("(doors) constrain from: {:?} to {:?}", from, to);
            let _ = cdt.add_constraint_edge(from, to);
        }

        let walls = self.walls_proj;
        for (from, to) in walls.into_iter() {
            println!("(wall) constrain from: {:?} to {:?}", from, to);
            let _ = cdt.add_constraint_edge(from, to);
        }

        /*let obstacles: Vec<_> = self.bboxes_proj;
        for obstacle_bound in obstacles.into_iter() {
            println!("Constrain cycle: {:?}", obstacle_bound);
            let _ = cdt.add_constraint_edges(obstacle_bound, true);
        }*/

        cdt
    }
}

pub fn walls_doors_ground_projection(walls: &Vec<Wall>, doors: &Vec<Door>) -> [Vec<(Point2<f32>, Point2<f32>)>; 2] {
    let mut projected_walls = Vec::new();
    let mut projected_doors = Vec::new();

    for wall in walls.iter() {
        let start: Point2<f32> = (wall.start.x, wall.start.y).into();
        let end: Point2<f32> = (wall.end.x, wall.end.y).into();

        let dir = point2_sub(end, start);
        let len = (dir.x * dir.x + dir.y * dir.y).sqrt();
        let unit = (dir.x / len, dir.y / len);

        let mut door_found = false;
        // doors on a single wall
        for door in doors.iter() {
            if door.wall0_id == wall.id {
                let center: Point2<f32> = (door.position.x, door.position.y).into();

                let offset: Point2<f32> = (unit.0 * door.width * 0.5, unit.1 * door.width * 0.5).into();


                projected_doors.push((point2_add(center, offset), point2_sub(center, offset)));
                projected_walls.push((start, point2_sub(center, offset)));
                projected_walls.push((point2_add(center, offset), end));
                door_found = true;
            }
        }
        if !door_found {
            projected_walls.push((start, end));
        }
    }

    [projected_doors, projected_walls]
}

fn obstacles_ground_projection(bboxes: &Vec<Bbox>) -> Vec<[Point2<f32>; 4]> {
    let mut obstacles = Vec::new();
    for bbox in bboxes.iter() {
        let hx = bbox.scale.x * 0.5;
        let hy = bbox.scale.y * 0.5;

        let c = bbox.angle.cos();
        let s = bbox.angle.sin();

        let local_corners = [
            (-hx, -hy),
            (-hx,  hy),
            ( hx,  hy),
            ( hx, -hy),
        ];

        let corners: [Point2<f32>; 4] = local_corners.map(|(x, y)| {
            (bbox.position.x + x * c - y * s, bbox.position.y + x * s + y * c ).into()
        });
        obstacles.push(corners);
    }
    obstacles
}
