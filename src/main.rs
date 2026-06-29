use std::env;

mod navigation;
mod device_stream;
mod sensor_data;
mod sina;

fn main() -> anyhow::Result<()> {

    if env::args().len() < 2 {
        println!("Missing room semantic file id");
        return Ok(());
    }
    let file_id = env::args().nth(1).unwrap();

    let prefix = String::from("input/");
    let suffix = String::from("/ase_scene_language.txt");
    let filepath = prefix + &file_id.as_str() + &suffix;

    let mut sina = sina::Sina::new();
    sina.launch(filepath)?;

    Ok(())
}
