use std::sync::mpsc;
use std::thread;
use nalgebra::Vector3;
use rerun::{RecordingStream, RecordingStreamBuilder};

use crate::{
    device_stream,
    msckf,
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
        let (imu_tx, image_tx, pos3_rx) = Self::start_msckf(record.clone());
        let pos2_tx = Self::start_navigator(record.clone(), semantic_path);

        // FIX: multiplexing with crossbeam to avoid the busy-spin loop
        loop {
            if let Ok(sensor_data) = sensor_rx.try_recv() {
                match sensor_data {
                    SensorData::Imu(imu) => imu_tx.send(imu)?,
                    SensorData::Image(image) => image_tx.send(image)?,
                }
            }
            if let Ok(pos3) = pos3_rx.try_recv() {
                let pos2: navigation::Point = (pos3.x, pos3.y).into();
                pos2_tx.send(pos2)?;
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

    fn start_msckf(record: RecordingStream) -> (
        mpsc::Sender<ImuMessage>,
        mpsc::Sender<ImageMessage>,
        mpsc::Receiver<Vector3<f64>>
        ) {
        let (imu_tx, imu_rx): (mpsc::Sender<ImuMessage>, mpsc::Receiver<ImuMessage>) = mpsc::channel();
        let (image_tx, image_rx): (mpsc::Sender<ImageMessage>, mpsc::Receiver<ImageMessage>) = mpsc::channel();
        let (pos_tx, pos_rx): (mpsc::Sender<Vector3<f64>>, mpsc::Receiver<Vector3<f64>>) = mpsc::channel();

        let _ = thread::Builder::new()
            .name("MSCKF thread".to_string())
            .spawn(move || msckf::MSCKF::new().launch(record, imu_rx, image_rx, pos_tx).unwrap());

        (imu_tx, image_tx, pos_rx)
    }

    fn start_navigator(record: RecordingStream, semantic_path: String) -> mpsc::Sender<navigation::Point> {
        let (tx, rx): (mpsc::Sender<navigation::Point>, mpsc::Receiver<navigation::Point>) = mpsc::channel();

        let _ = thread::Builder::new()
            .name("Navigator thread".to_string())
            .spawn(move || navigation::Navigator::new(semantic_path).launch(record, rx).unwrap());
        tx
    }
}
