use std::env;

mod navigation;
mod device_stream;
mod sensor_data;
mod pos_sys;
mod sina;
mod preprocessing;

fn main() -> anyhow::Result<()> {

    if env::args().len() < 2 {
        println!("Missing room semantic file id");
        return Ok(());
    }
    let file_id = env::args().nth(1).unwrap();

    let prefix = String::from("input/");
    let filepath = prefix + &file_id.as_str();

    sina::Sina::new().launch(filepath, (3.0, 5.0))?;

    Ok(())
}
