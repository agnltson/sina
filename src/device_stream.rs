use std::process::{Command, Stdio};
use std::sync::mpsc;

use crate::sensor_data::{SensorData, ImuMessage, ImageMessage};

use zmq;
use serde_json::Value;

pub struct DeviceStream<'a> {
    stream_args: Vec<&'a str>,
}

impl<'a> DeviceStream<'a> {
    pub fn new(stream_args: Vec<&'a str>) -> Self {
        Self {
            stream_args,
        }
    }

    pub fn launch(&self, sensor_data_tx: mpsc::Sender<SensorData>) -> anyhow::Result<()> {
        println!("Launching sensor data stream");
        let mut child = Command::new("python")
            .arg("stream/device_stream.py")
            .args(&self.stream_args)
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()?;
        let ctx = zmq::Context::new();
        let socket = ctx.socket(zmq::SUB)?;

        socket.connect("tcp://localhost:5555")?;
        socket.set_subscribe(b"")?;

        loop {
            let msg = socket.recv_string(0)?.unwrap();

            let v: Value = serde_json::from_str(&msg)?;

            match v["type"].as_str() {
                Some("imu") => {
                    let imu = ImuMessage::from_json(&msg)?;
                    // We only send from imu 0 it's the fastest one
                    if imu.imu_idx == 0 {
                        let sd: SensorData = SensorData::Imu(imu);
                        sensor_data_tx.send(sd)?;
                    }                }

                Some("slam_image") => {
                    let sd: SensorData = SensorData::Image(ImageMessage::from_json(&msg)?);
                    sensor_data_tx.send(sd)?;
                }

                _ => {
                    eprintln!("unknown message");
                }
            }
        }

        child.kill()?;
        child.wait()?;
        Ok(())
    }
}
