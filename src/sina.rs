use std::sync::mpsc;
use std::thread;
use nalgebra::Vector3;
use rerun::{RecordingStream, RecordingStreamBuilder, Points2D, Color};
use std::process::{Command, Stdio};
use futures::StreamExt;

use crate::{
    navigation,
};

pub struct Sina {
}

impl Sina {
    pub fn new() -> Self {
        Self {
        }
    }

    pub fn launch(&mut self, semantic_path: String) -> anyhow::Result<()> {
        let record: RecordingStream = RecordingStreamBuilder::new("SINA").spawn()?;

        Self::start_openvins()?;
        Self::start_python_publisher()?;

        let pos_rx = Self::start_odometry_subscriber();
        let pos_tx = Self::start_navigator(record, semantic_path);

        loop {
            if let Ok(pos) = pos_rx.try_recv() {
                println!("received pos: {:?}", pos);
            }
        }

        Ok(())
    }

    fn start_openvins() -> anyhow::Result<()> {
        let _openvins = Command::new("bash")
            .arg("-c")
            .arg("source /opt/ros/humble/setup.bash && \
                  source /root/ov_ws/install/setup.bash && \
                  ros2 launch ov_msckf subscribe.launch.py \
                  config:=/root/sina/aria_openVINS.yaml")
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()?;
        Ok(())
    }

    fn start_python_publisher() -> anyhow::Result<()> {
        let _ = Command::new("bash")
            .arg("-c")
            .arg("source /opt/ros/humble/setup.bash && \
                  python3 stream/device_stream.py \
                  --interface wifi \
                  --device-ip 10.69.83.218 \
                  --profile profile14")
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()?;
        Ok(())
    }

    fn start_odometry_subscriber() -> mpsc::Receiver<Vector3<f64>> {
        let (tx, rx) = mpsc::channel();
        thread::Builder::new()
            .name("r2r odometry thread".to_string())
            .spawn(move || run_odometry_subscriber(tx).unwrap())
            .unwrap();
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

fn run_odometry_subscriber(tx: mpsc::Sender<Vector3<f64>>) -> anyhow::Result<()> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "sina_odom_subscriber", "")?;

    let mut sub = node.subscribe::<r2r::nav_msgs::msg::Odometry>(
        "/ov_msckf/odomimu",
        r2r::QosProfile::default(),
    )?;

    let mut node_for_spin = node;
    thread::spawn(move || loop {
        node_for_spin.spin_once(std::time::Duration::from_millis(10));
    });

    let rt = tokio::runtime::Runtime::new()?;
    rt.block_on(async move {
        while let Some(msg) = sub.next().await {
            let pos = extract_position(&msg);
            if tx.send(pos).is_err() {
                break;
            }
        }
    });

    Ok(())
}

fn extract_position(msg: &r2r::nav_msgs::msg::Odometry) -> Vector3<f64> {
    let p = &msg.pose.pose.position;
    Vector3::new(p.x, p.y, p.z)
}
