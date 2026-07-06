use std::sync::mpsc;
use rerun::{RecordingStream, Color, Points2D};

use super::{
    navgraph::NavGraph,
    path::Path,
    Point,
};

pub struct Navigator {
    position: Option<Point>,
    heading: Option<Point>,
    navgraph: NavGraph,
    video_path: Path,
}

impl Navigator {
    pub fn new(filepath: String) -> Self {
        Self {
            position: None,
            heading: None,
            navgraph: NavGraph::new(&filepath),
            video_path: Path::from_closed_loop(&filepath),
        }
    }

    pub fn compute_path(&self, start: (f64, f64), end: (f64, f64)) -> Option<Path> {
        let point_path = self.navgraph.find_path(start.into(), end.into())?;
        Some(Path::from_points(point_path))
    }

    pub fn launch(&mut self, record: RecordingStream, pos_rx: mpsc::Receiver<(Point, Point)>) -> anyhow::Result<()> {
        self.log_plan(&record, "navigator")?;
        self.log_videopath(&record, "navigator")?;
        loop {
            if let Ok((pos, head)) = pos_rx.recv() {
                self.position = Some(pos);
                self.heading = Some(head);
                self.log_position(&record, "navigator/position")?;
                self.log_heading(&record, "navigator/position")?;
            }
        }
        Ok(())
    }

    fn log_plan(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.navgraph.log(
            record,
            format!("{}/plan", log_path).as_str(),
            )?;
        Ok(())
    }

    fn log_videopath(&self, record: &RecordingStream, log_path: &str) -> anyhow::Result<()> {
        self.video_path.log(
            record,
            format!("{}/video_path", log_path).as_str(),
            )?;
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
            let (px, py): (f64, f64) = pos.into();
            let (hx, hy): (f64, f64) = heading.into();

            let end_x = px + hx;
            let end_y = py + hy;

            record.log(
                format!("{}/heading", log_path).as_str(),
                &rerun::LineStrips2D::new([[
                    [px as f32, py as f32],
                    [end_x as f32, end_y as f32],
                ]])
                .with_colors([Color::from_rgb(0, 255, 0)]),
            )?;
        }

        Ok(())
    }
}

