use std::process::{Command, Stdio};

use crate::sensor_data::{ImuMessage, ImageMessage};

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

    pub fn start(&self) -> Result<(), Box<dyn std::error::Error>> {
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
                    let m: ImuMessage = ImuMessage::from_json(&msg)?;
                    println!("{:?}", m);
                }

                Some("slam_image") => {
                    let m: ImageMessage = ImageMessage::from_json(&msg)?;
                    println!("image received ({} bytes)", msg.len());
                }

                _ => {
                    eprintln!("unknown message");
                }
            }
        }
        child.kill()?;
    }
}
