use std::sync::mpsc;
use std::thread;
use nalgebra::Vector3;
use rerun::{RecordingStream, RecordingStreamBuilder, Points2D, Color};

use crate::{
    device_stream,
    navigation,
};
use crate::sensor_data::{StateMessage};

pub struct Sina {
}

impl Sina {
    pub fn new() -> Self {
        Self {
        }
    }

    pub fn launch(&mut self, semantic_path: String) -> anyhow::Result<()> {
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;

        let rawstate_rx = Self::start_sensor_stream();
        let state_tx = Self::start_navigator(record.clone(), semantic_path);

        loop {
            if let Ok(message) = rawstate_rx.try_recv() {
                let pos: navigation::Point = (message.position[0], message.position[1]).into();
                let dir: navigation::Point = (message.rotation[0], message.rotation[1]).into();
                state_tx.send((pos, dir))?;
            }
        }

        Ok(())
    }

    fn start_sensor_stream() -> mpsc::Receiver<StateMessage> {
        let (tx, rx): (mpsc::Sender<StateMessage>, mpsc::Receiver<StateMessage>) = mpsc::channel();

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

    fn start_navigator(record: RecordingStream, semantic_path: String) -> mpsc::Sender<(navigation::Point, navigation::Point)> {
        let (tx, rx): (mpsc::Sender<(navigation::Point, navigation::Point)>, mpsc::Receiver<(navigation::Point, navigation::Point)>) = mpsc::channel();

        let _ = thread::Builder::new()
            .name("Navigator thread".to_string())
            .spawn(move || navigation::Navigator::new(semantic_path).launch(record, rx).unwrap());
        tx
    }
}
