use std::process::{Command, Stdio};
use std::sync::mpsc::Sender;

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

    pub fn run(&self, sensor_data_sender: Sender<SensorData>) -> Result<(), Box<dyn std::error::Error>> {
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
                    let sd: SensorData = SensorData::Imu(ImuMessage::from_json(&msg)?);
                    if sensor_data_sender.send(sd).is_err() {
                        break;
                    }
                }

                Some("slam_image") => {
                    let sd: SensorData = SensorData::Image(ImageMessage::from_json(&msg)?);
                    if sensor_data_sender.send(sd).is_err() {
                        break;
                    }
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
