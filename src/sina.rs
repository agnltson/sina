use std::sync::mpsc;
use std::thread;
use nalgebra::Vector3;
use rerun::{RecordingStream, RecordingStreamBuilder, Points2D, Color};

use crate::{
    device_stream,
    navigation,
};
use crate::sensor_data::{SensorData, ImuMessage, ImageMessage};

pub struct Sina {
}

impl Sina {
    pub fn new() -> Self {
        Self {
        }
    }

    pub fn launch(&mut self, semantic_path: String) -> anyhow::Result<()> {
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;

        let sensor_rx = Self::start_sensor_stream();
        let _ = Self::start_navigator(record, semantic_path);

        // FIX: multiplexing with crossbeam to avoid the busy-spin loop
        loop {
            if let Ok(data) = sensor_rx.try_recv() {
                println!("received data: {:?}", data);
            }
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

    fn start_navigator(record: RecordingStream, semantic_path: String) -> mpsc::Sender<navigation::Point> {
        let (tx, rx): (mpsc::Sender<navigation::Point>, mpsc::Receiver<navigation::Point>) = mpsc::channel();

        let _ = thread::Builder::new()
            .name("Navigator thread".to_string())
            .spawn(move || navigation::Navigator::new(semantic_path).launch(record, rx).unwrap());
        tx
    }
}
