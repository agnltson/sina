use std::sync::mpsc;
use rerun::{
    RecordingStream,
    Color,
    Points2D,
    Arrows2D,
};

use super::{
    navgraph::NavGraph,
    path::Path,
    Point,
};

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

    pub fn new(filepath: String) -> Self {
        Self {
            position: None,
            heading: None,
            navgraph: NavGraph::new(&filepath),
            path: None,
            path_idx: 0,
        }
    }

    pub fn compute_path(&self, start: Point, end: Point) -> Option<Path> {
        let point_path = self.navgraph.find_path(start, end)?;
        Some(Path::from_points(point_path))
    }

    pub fn launch(&mut self, record: RecordingStream, pos_rx: mpsc::Receiver<(Point, Point)>, goal: (f64, f64)) -> anyhow::Result<()> {
        self.log_plan(&record, "navigator")?;
        let goal_point: Point = goal.into();

        loop {
            match pos_rx.recv() {
                Ok((pos, head)) => {
                    self.position = Some(pos);
                    self.heading = Some(head);


                    if self.need_replan(pos) {
                        self.path = self.compute_path(pos, goal_point);
                        self.path_idx = 0;
                    }

                    self.log_path(&record, "navigator")?;
                    self.log_position(&record, "navigator/position")?;
                    self.log_heading(&record, "navigator/position")?;
                },
                Err(_) => {
                    eprintln!("[Navigator] channel de position fermé, arrêt.");
                    break;
                }
            }
        }
        Ok(())
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

    fn log_plan(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.navgraph.log(
            record,
            format!("{}/plan", log_path).as_str(),
            )?;
        Ok(())
    }

    fn log_path(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        if let Some(path) = &self.path {
            path.log(
                record,
                format!("{}/path", log_path).as_str(),
                )?;
        }
        Ok(())
    }

    fn log_position(
        &self,
        record: &RecordingStream,
        log_path: &str,
    ) -> anyhow::Result<()> {
        if let Some(pos) = self.position {
            let (x, y): (f64, f64) = pos.into();

            record.log(
                format!("{}/position", log_path).as_str(),
                &Points2D::new([[x as f32, y as f32]])
                    .with_colors([Color::from_rgb(255, 0, 0)])
                    .with_radii([0.15]),
            )?;
        }

        Ok(())
    }

    fn log_heading(
        &self,
        record: &RecordingStream,
        log_path: &str,
    ) -> anyhow::Result<()> {
        if let (Some(pos), Some(heading)) = (self.position, self.heading) {
            record.log(
                format!("{}/heading", log_path).as_str(),
                &Arrows2D::from_vectors([[heading.x.into_inner(), heading.y.into_inner()]])
                    .with_origins([[pos.x.into_inner(), pos.y.into_inner()]])
                    .with_colors([Color::from_rgb(0, 255, 0)]),
            )?;
        }
        Ok(())
    }
}

