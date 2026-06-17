use std::{env, sync::mpsc};
use rerun::RecordingStreamBuilder;

mod navigation;
mod msckf;
mod device_stream;
mod sensor_data;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    if env::args().len() < 2 {
        println!("Missing room semantic file id");
        return Ok(());
    }
    let file_id = env::args().nth(1).unwrap();

    let prefix = String::from("input/");
    let suffix = String::from("/ase_scene_language.txt");
    let filepath = prefix + &file_id.as_str() + &suffix;

    let nav = navigation::Navigator::new(&filepath);
    nav.display()?;

    let stream_args = vec![
        "--interface",
        "wifi",
        "--device-ip",
        "10.178.117.218",
    ];

    let streamer = device_stream::DeviceStream::new(stream_args);
    //streamer.start()?;

    Ok(())
}
