use std::env;

mod navigation;
mod sina;

fn main() -> anyhow::Result<()> {

    if env::args().len() < 2 {
        println!("Missing room semantic file id");
        return Ok(());
    }
    let file_id = env::args().nth(1).unwrap();

    let prefix = String::from("input/");
    let filepath = prefix + &file_id.as_str();

    let mut sina = sina::Sina::new(filepath);
    sina.launch((-0.5, -3.0), (3.0, 5.0))?;

    Ok(())
}
