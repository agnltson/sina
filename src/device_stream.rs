use std::process::{Command, Stdio};
use std::sync::mpsc;

use crate::sensor_data::{StateMessage};

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

    pub fn launch(&self, rawstate_tx: mpsc::Sender<StateMessage>) -> anyhow::Result<()> {
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
            let st: StateMessage = StateMessage::from_json(&msg)?;
            rawstate_tx.send(st)?;
        }

        child.kill()?;
        child.wait()?;
        Ok(())
    }
}
