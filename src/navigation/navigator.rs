use std::{
    sync::mpsc,
    time::Duration,
    thread,
};

use super::{
    navgraph::NavGraph,
    path::Path,
    Point,
};
use crate::rendering;

pub struct Navigator {
    position: Option<Point>,
    heading: Option<Point>,
    navgraph: NavGraph,
    path: Option<Path>,
    path_idx: usize,
}

impl Navigator {
    const REPLAN_THRESHOLD: f32 = 1.5;
    const SEARCH_WINDOW: usize = 5;
    const POLL_INTERVAL: Duration = Duration::from_millis(10);

    pub fn new(filepath: String) -> Self {
        let navgraph = NavGraph::new(&filepath);
        let node_positions = navgraph.get_node_positions();

        Self {
            position: None,
            heading: None,
            navgraph,
            path: None,
            path_idx: 0,
        }
    }

    pub fn compute_path(&self, start: Point, end: Point) -> Option<Path> {
        let point_path = self.navgraph.find_path(start, end)?;
        Some(Path::from_points(point_path))
    }

    pub fn launch(
        &mut self,
        pos_rx: mpsc::Receiver<(Point, Point)>,
        render_tx: mpsc::Sender<rendering::RenderUpdate>,
        click_rx: mpsc::Receiver<Point>,
    ) -> anyhow::Result<()> {
        let _ = render_tx.send(rendering::RenderUpdate::Static(self.static_geometry()));

        let mut goal_point: Option<Point> = None;
        let mut pos_disconnected = false;

        loop {
            let mut dirty = false;

            match pos_rx.try_recv() {
                Ok((pos, head)) => {
                    self.position = Some(pos);
                    self.heading = Some(head);
                    dirty = true;

                    if let Some(goal) = goal_point {
                        if goal_point.is_some() && self.need_replan(pos) {
                            self.path = self.compute_path(pos, goal);
                            self.path_idx = 0;
                        }
                    }
                },
                Err(mpsc::TryRecvError::Empty) => {},
                Err(mpsc::TryRecvError::Disconnected) => {
                    eprintln!("[Navigator] position channel closed, shutting down.");
                    pos_disconnected = true;
                    break;
                }
            }

            match click_rx.try_recv() {
                Ok(clicked_goal) => {
                    goal_point = Some(clicked_goal);
                    if let Some(pos) = self.position {
                        self.path = self.compute_path(pos, clicked_goal);
                        self.path_idx = 0;
                        dirty = true;
                    }
                },
                Err(mpsc::TryRecvError::Empty) => {},
                Err(mpsc::TryRecvError::Disconnected) => {
                    eprintln!("[Navigator] navigation window closed shutting down.");
                    break;
                }
            }

            if dirty {
                let _ = render_tx.send(rendering::RenderUpdate::Dynamic(self.dynamic_state()));
            }

            thread::sleep(Self::POLL_INTERVAL);
        }
        Ok(())
    }

    fn static_geometry(&self) -> rendering::StaticGeometry {
        rendering::StaticGeometry {
            walls: self.navgraph.wall_segments(),
            doors: self.navgraph.door_segments(),
            bboxes: self.navgraph.bbox_polygons(),
            borders: if cfg!(debug_assertions) { self.navgraph.border_polygons() } else { Vec::new() },
            holes: if cfg!(debug_assertions) { self.navgraph.hole_polygons() } else { Vec::new() },
            navmesh_polygons: if cfg!(debug_assertions) { self.navgraph.polygon_vertices() } else { Vec::new() },
            navgraph_edges: if cfg!(debug_assertions) { self.navgraph.edges_as_points() } else { Vec::new() },
        }
    }

    fn dynamic_state(&self) -> rendering::DynamicState {
        rendering::DynamicState {
            position: self.position,
            heading: self.heading,
            path: self.path.as_ref().map(|p| p.points().to_vec()), // voir note plus bas
        }
    }

    fn need_replan(&mut self, pos: Point) -> bool {
        match &self.path {
            None => true,
            Some(path) => match path.closest_segment(pos, self.path_idx, Self::SEARCH_WINDOW) {
                Some((idx, _t, dist)) => {
                    self.path_idx = idx;
                    dist > Self::REPLAN_THRESHOLD
                }
                None => true,
            },
        }
    }
}

