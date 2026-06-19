use std::sync::mpsc;
use std::thread;
use nalgebra::Vector3;
use rerun::{RecordingStream, RecordingStreamBuilder};

use crate::{
    device_stream,
    msckf,
    navigation,
};
use crate::sensor_data::SensorData;
use crate::sensor_buffer::SensorBuffer;

pub struct Sina {
    buffer: SensorBuffer,
}

impl Sina {
    pub fn new() -> Self {
        Self {
            buffer: SensorBuffer::new(),
        }
    }

    pub fn launch(&mut self, semantic_path: String) -> anyhow::Result<()> {
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;

        let sensor_rx = Self::start_sensor_stream();
        let (buffer_tx, pos3_rx) = Self::start_msckf(record.clone());
        let pos2_tx = Self::start_navigator(record.clone(), semantic_path);

        // FIX: multiplexing with crossbeam to avoid the busy-spin loop
        loop {
            if let Ok(sensor_data) = sensor_rx.try_recv() {
                self.handle_sensor_data(sensor_data);
                if self.buffer.is_ready_to_process() {
                    buffer_tx.send(self.buffer.clone())?;
                    self.buffer.clear();
                }
            }
            if let Ok(pos3) = pos3_rx.try_recv() {
                let pos2: navigation::Point = (pos3.x, pos3.y).into();
                pos2_tx.send(pos2)?;
            }
        }

        Ok(())
    }

    // TODO: using all sensor (cf sensor_buffer)
    fn handle_sensor_data(&mut self, sensor_data: SensorData) {
        match sensor_data {
            SensorData::Imu(m) => {
                if m.imu_idx == 1 {
                    self.buffer.push_imu(m);
                }
            },
            SensorData::Image(m) => {
                if m.camera == 1 {
                    self.buffer.push_image(m);
                }
            },
        }
    }

    fn start_sensor_stream() -> mpsc::Receiver<SensorData> {
        let (tx, rx): (mpsc::Sender<SensorData>, mpsc::Receiver<SensorData>) = mpsc::channel();

        let stream_args = vec![
            "--interface",
            "wifi",
            "--device-ip",
            "10.178.117.218",
            "--profile",
            "profile14",
        ];

        let _ = thread::Builder::new()
            .name("Sensor data streaming thread".to_string())
            .spawn(move || device_stream::DeviceStream::new(stream_args).launch(tx).unwrap());

        rx
    }

    fn start_msckf(record: RecordingStream) -> (mpsc::Sender<SensorBuffer>, mpsc::Receiver<Vector3<f64>>) {
        let (buffer_tx, buffer_rx): (mpsc::Sender<SensorBuffer>, mpsc::Receiver<SensorBuffer>) = mpsc::channel();
        let (pos_tx, pos_rx): (mpsc::Sender<Vector3<f64>>, mpsc::Receiver<Vector3<f64>>) = mpsc::channel();

        let _ = thread::Builder::new()
            .name("MSCKF thread".to_string())
            .spawn(move || msckf::MSCKF::new().launch(record, buffer_rx, pos_tx).unwrap());

        (buffer_tx, pos_rx)
    }

    fn start_navigator(record: RecordingStream, semantic_path: String) -> mpsc::Sender<navigation::Point> {
        let (tx, rx): (mpsc::Sender<navigation::Point>, mpsc::Receiver<navigation::Point>) = mpsc::channel();

        let _ = thread::Builder::new()
            .name("Navigator thread".to_string())
            .spawn(move || navigation::Navigator::new(semantic_path).launch(record, rx).unwrap());
        tx
    }
}
