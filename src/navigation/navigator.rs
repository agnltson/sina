use rerun::RecordingStreamBuilder;

use super::navgraph::NavGraph;
use super::utils::Point;

pub struct Navigator {
     navgraph: NavGraph,
     position: Option<Point>,
     path: Option<Vec<usize>>,
}

impl Navigator {
    pub fn new(filepath: &str) -> Self {
        Self {
            navgraph: NavGraph::new(filepath),
            position: Some((0.0, 0.0).into()),
            path: None,
        }
    }

    pub fn display(&self) -> Result<(), Box<dyn std::error::Error>>{
        let rec = RecordingStreamBuilder::new("pathfinding")
            .spawn()?;

        let log_path = "navigator/";

        let _ = self.navgraph.render_rerun(&rec, log_path);
        if let Some(path) = &self.path {
            let _ = self.navgraph.render_rerun_path(&path, &rec, log_path);
        }
        if let Some(pos) = &self.position {
            let _ = pos.render_rerun(&rec, &(String::from("position/") + log_path));
        }
        Ok(())
    }
}
