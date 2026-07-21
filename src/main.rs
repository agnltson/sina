mod config;
mod navigation;
mod device_stream;
mod sensor_data;
mod pos_sys;
mod sina;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// Path to configuration file
    #[arg(long, default_value = "config/default.toml")]
    config: String,

    /// Launch navigation mode with the input inside given folder.
    /// The folder must contain:
    /// semantic.txt (Scenescript output),
    /// tags_world_config.json
    #[arg(long)]
    nav: Option<String>,
}

fn main() -> anyhow::Result<()> {

    std::panic::set_hook(Box::new(|info| {
        let thread = std::thread::current();
        let name = thread.name().unwrap_or("unnamed");
        eprintln!("!!! PANIC in thread '{name}': {info}");
    }));

    let args = Args::parse();

    let config_path = args.config;
    let config = config::load_config(config_path)?;
    if let Some(nav_path) = args.nav {
        println!("Launching navigation : {}", nav_path);
        sina::Sina::new().launch(&config, nav_path, (3.0, 5.0))?;
    }

    Ok(())
}
