use std::{
    fs::File,
    io::{
        BufRead,
        BufReader,
    }
};
use rerun::{Points2D, LineStrips2D, RecordingStream, Color};
use super::Point;

#[derive(Debug)]
pub struct Path {
    pos: Vec<Point>,
}

impl Path {
    pub fn new(filepath: &str) -> Self {
        let source_name = "trajectory.csv";
        let full_path = format!("{}/{}", filepath, source_name);

        let file = File::open(&full_path)
            .unwrap_or_else(|e| panic!("Unable to open {}: {}", full_path, e));
        let reader = BufReader::new(file);

        let mut pos = Vec::new();

        for (i, line) in reader.lines().enumerate() {
            let line = line.unwrap_or_else(|e| panic!("Reading error line {}: {}", i, e));

            if i == 0 {
                println!("first line: {}", line);
                continue;
            }

            let fields: Vec<&str> = line.split(',').collect();

            let x: f64 = fields[3]
                .parse()
                .unwrap_or_else(|e| panic!("Error while parsing x line {}: {}", i, e));
            let y: f64 = fields[4]
                .parse()
                .unwrap_or_else(|e| panic!("Error while parsing y line {}: {}", i, e));

            pos.push((x, y).into());
        }

        Path { pos }
    }

    pub fn log(
        &self,
        rec: &RecordingStream,
        log_path: &str,
        ) -> anyhow::Result<()> {
        let points: Vec<[f32; 2]> = self
            .pos
            .iter()
            .map(|p| [p.x.into_inner(), p.y.into_inner()])
            .collect();

        rec.log(
            format!("{}/path/points", log_path).as_str(),
            &Points2D::new(points.clone()),
        )?;

        if points.len() >= 2 {
            rec.log(
                format!("{}/path/line", log_path).as_str(),
                &LineStrips2D::new(vec![points])
                    .with_colors([Color::from_rgb(255, 165, 0)]), // orange
            )?;
        }

        Ok(())
    }
}
