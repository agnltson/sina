use std::env;
use rerun::RecordingStreamBuilder;

mod navigation;

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

    Ok(())
}
