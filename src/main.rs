use std::{env, sync::mpsc, thread};
use rerun::RecordingStreamBuilder;
use std::sync::mpsc::{Sender, Receiver};
use nalgebra::Vector3;

mod navigation;
mod msckf;
mod device_stream;
mod sensor_data;

use sensor_data::SensorData;
use navigation::VisualisationData;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    if env::args().len() < 2 {
        println!("Missing room semantic file id");
        return Ok(());
    }
    let file_id = env::args().nth(1).unwrap();

    let prefix = String::from("input/");
    let suffix = String::from("/ase_scene_language.txt");
    let filepath = prefix + &file_id.as_str() + &suffix;

    let stream_args = vec![
        "--interface",
        "wifi",
        "--device-ip",
        "10.178.117.218",
        "--profile",
        "profile14",
    ];

    // Channels
    let (sensor_data_sender, sensor_data_receiver): (Sender<SensorData>, Receiver<SensorData>) = mpsc::channel();
    let (visual_data_sender, visual_data_receiver): (Sender<VisualisationData>, Receiver<VisualisationData>) = mpsc::channel();

    // --- Sensor data streaming ---

    let streamer = device_stream::DeviceStream::new(stream_args);

    let stream_thread = thread::Builder::new()
        .name("Sensor data streaming".to_string())
        .spawn(move || streamer.run(sensor_data_sender).unwrap());

    // --- MSCKF ---
    let mut msckf = msckf::MSCKF::new();
    let msckf_thread = thread::Builder::new()
        .name("MSCKF computing".to_string())
        .spawn(move || msckf.run(sensor_data_receiver, visual_data_sender).unwrap());

    // --- Navigator/Display ---
    let mut nav = navigation::Navigator::new(&filepath);

    nav.run(visual_data_receiver)?;

    Ok(())
}
