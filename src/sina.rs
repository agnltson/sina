use std::{
    thread,
    sync::mpsc,
    collections::HashMap,
};
use rerun::{RecordingStream, RecordingStreamBuilder};
use nalgebra::Vector3;

use crate::{
    navigation,
    device_stream,
    sensor_data::{
        SensorData,
    },
    pos_sys,
};

pub struct Sina {
}

impl Sina {
    pub fn new() -> Self {
        Self {}
    }

    pub fn launch(
        &mut self,
        semantic_path: String,
        start: (f64, f64),
        end: (f64, f64)
        ) -> anyhow::Result<()> {
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;

        let sensor_rx = Self::start_sensor_stream();
        let (sensor_tx, pos_rx) = Self::start_positioning_system(record.clone());
        let point_tx = Self::start_navigator(record, semantic_path);

        loop {
            if let Ok(sensor_data) = sensor_rx.try_recv() {
                sensor_tx.send(sensor_data)?;
            }
            if let Ok(pos3) = pos_rx.try_recv() {
                let point: navigation::Point = (pos3.x, pos3.y).into();
                point_tx.send(point)?;
            }
        }

        Ok(())
    }

    fn start_positioning_system(record: RecordingStream) -> (mpsc::Sender<SensorData>, mpsc::Receiver<Vector3<f64>>) {
        let (sensor_tx, sensor_rx): (mpsc::Sender<SensorData>, mpsc::Receiver<SensorData>) = mpsc::channel();
        let (position_tx, position_rx): (mpsc::Sender<Vector3<f64>>, mpsc::Receiver<Vector3<f64>>) = mpsc::channel();

        let _ = thread::Builder::new()
            .name("Positioning system thread".to_string())
            .spawn(move || pos_sys::PosSys::new().launch(record, sensor_rx, position_tx).unwrap());

        (sensor_tx, position_rx)
    }

    fn start_sensor_stream() -> mpsc::Receiver<SensorData> {
        let (tx, rx): (mpsc::Sender<SensorData>, mpsc::Receiver<SensorData>) = mpsc::channel();

        let stream_args = vec![
            "--interface",
            "wifi",
            "--device-ip",
            "10.69.83.218",
            "--profile",
            "profile12",
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
