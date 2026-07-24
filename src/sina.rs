use std::{
    thread,
    sync::mpsc,
    sync::mpsc::TryRecvError,

};
use nalgebra::{
    Vector2,
    Vector3,
    UnitQuaternion,
};

use crate::{
    navigation,
    device_stream,
    sensor_data::{
        SensorData,
    },
    pos_sys,
    config::Config,
    rendering,
};

pub struct Sina {
}

impl Sina {
    pub fn new() -> Self {
        Self {}
    }

    pub fn launch(
        &mut self,
        config: &Config,
        data_path: String,
        ) -> anyhow::Result<()> {

        let sensor_rx = Self::start_sensor_stream(config.clone());
        let (sensor_tx, pos_rx) = Self::start_positioning_system(config.clone(), data_path.clone());
        let (pos_tx, render_rx, click_tx) = Self::start_navigator(data_path);
        Self::start_data_bridge(sensor_rx, sensor_tx, pos_rx, pos_tx);

        rendering::render(render_rx, click_tx)?;

        Ok(())
    }

    fn start_data_bridge(
        sensor_rx: mpsc::Receiver<SensorData>,
        sensor_tx: mpsc::Sender<SensorData>,
        pos_rx: mpsc::Receiver<(Vector3<f64>, UnitQuaternion<f64>)>,
        pos_tx: mpsc::Sender<(navigation::Point, navigation::Point)>,
    ) {
        println!("Launching Data bridge thread");
        let _ = thread::Builder::new()
            .name("Data bridge".to_string())
            .spawn(move || {
                loop {
                    match sensor_rx.try_recv() {
                        Ok(sensor_data) => {
                            if sensor_tx.send(sensor_data).is_err() {
                                eprintln!("[Sina] positioning system closed (sensor_tx), shutting down.");
                                break;
                            }
                        }
                        Err(TryRecvError::Disconnected) => {
                            eprintln!("[Sina] sensor stream closeds, shutting down.");
                            break;
                        }
                        Err(TryRecvError::Empty) => {}
                    }

                    match pos_rx.try_recv() {
                        Ok((pos3, orient4)) => {
                            let pos_point: navigation::Point = (pos3.x, pos3.y).into();
                            let heading = camera_heading_xy(&orient4);
                            if pos_tx.send((pos_point, heading)).is_err() {
                                eprintln!("[Sina] navigator closed (pos_tx), shutting down.");
                                break;
                            }
                        }
                        Err(TryRecvError::Disconnected) => {
                            eprintln!("[Sina] positioning system closed, shutting down.");
                            break;
                        }
                        Err(TryRecvError::Empty) => {}
                    }
                }
                eprintln!("[Sina] main loop stopped, end of program.");
            });
    }

    fn start_positioning_system(
        config: Config,
        data_path: String,
    ) -> (mpsc::Sender<SensorData>, mpsc::Receiver<(Vector3<f64>, UnitQuaternion<f64>)>) {
        let (sensor_tx, sensor_rx): (mpsc::Sender<SensorData>, mpsc::Receiver<SensorData>) = mpsc::channel();
        let (position_tx, position_rx):
            (mpsc::Sender<(Vector3<f64>, UnitQuaternion<f64>)>, mpsc::Receiver<(Vector3<f64>, UnitQuaternion<f64>)>)
             = mpsc::channel();

        println!("Launching Positioning system thread");
        let _ = thread::Builder::new()
            .name("Positioning system thread".to_string())
            .spawn(move || {
                if let Err(e) = pos_sys::PosSys::new(config, data_path).launch(sensor_rx, position_tx) {
                    eprintln!("[PosSys thread] fatal error : {e:?}");
                }
            });

        (sensor_tx, position_rx)
    }

    fn start_sensor_stream(config: Config) -> mpsc::Receiver<SensorData> {
        let (tx, rx): (mpsc::Sender<SensorData>, mpsc::Receiver<SensorData>) = mpsc::channel();

        let stream_args = vec![
            "--interface".to_string(),
            "wifi".to_string(),
            "--device-ip".to_string(),
            config.streaming.ip.clone(),
            "--profile".to_string(),
            config.streaming.profile.clone(),
        ];

        println!("Launching Sensor data streaming thread");
        let _ = thread::Builder::new()
            .name("Sensor data streaming thread".to_string())
            .spawn(move || {
                if let Err(e) = device_stream::DeviceStream::new(stream_args).launch(tx) {
                    eprintln!("[DeviceStream thread] fatal error : {e:?}");
                }
            });

        rx
    }

    fn start_navigator(
        data_path: String,
    ) -> (
        mpsc::Sender<(navigation::Point, navigation::Point)>,
        mpsc::Receiver<rendering::RenderUpdate>,
        mpsc::Sender<navigation::Point>,
    ) {
        let (pos_tx, pos_rx) = mpsc::channel();
        let (render_tx, render_rx) = mpsc::channel();
        let (click_tx, click_rx) = mpsc::channel();

        println!("Launching Navigator thread");
        let _ = thread::Builder::new()
            .name("Navigator thread".to_string())
            .spawn(move || {
                if let Err(e) = navigation::Navigator::new(data_path)
                    .launch(pos_rx, render_tx, click_rx)
                {
                    eprintln!("[Navigator thread] fatal error : {e:?}");
                }
            });

        (pos_tx, render_rx, click_tx)
    }
}

fn camera_heading_xy(orientation: &UnitQuaternion<f64>) -> navigation::Point {
    let forward_world = orientation.transform_vector(&Vector3::new(0.0, 0.0, 1.0));

    let heading = Vector2::new(forward_world.x, forward_world.y);
    let norm = heading.norm();

    if norm > 1e-6 {
        let res = heading / norm;
        (res.x, res.y).into()
    } else {
        (1.0, 0.0).into()
    }
}
