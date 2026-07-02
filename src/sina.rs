use std::{
    thread,
    sync::mpsc,
};
use rerun::{RecordingStream, RecordingStreamBuilder};

use crate::{
    navigation,
    device_stream,
    sensor_data::{
        SensorData,
    },
};

pub struct Sina {
    navigator: navigation::Navigator,
}

impl Sina {
    pub fn new(semantic_path: String) -> Self {
        Self {
            navigator: navigation::Navigator::new(semantic_path),
        }
    }

    pub fn launch(&mut self, start: (f64, f64), end: (f64, f64)) -> anyhow::Result<()> {
        let _ = Self::start_sensor_stream();
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;
        self.navigator.launch(&record)?;
        if let Some(p) = self.navigator.compute_path(start, end) {
            p.log(&record, "path")?;
        }
        Ok(())
    }

    fn start_sensor_stream() -> mpsc::Receiver<SensorData> {
        let (tx, rx): (mpsc::Sender<SensorData>, mpsc::Receiver<SensorData>) = mpsc::channel();

        let stream_args = vec![
            "--interface",
            "wifi",
            "--device-ip",
            "10.69.83.218",
            "--profile",
            "profile14",
        ];

        let _ = thread::Builder::new()
            .name("Sensor data streaming thread".to_string())
            .spawn(move || device_stream::DeviceStream::new(stream_args).launch(tx).unwrap());

        rx
    }
}
