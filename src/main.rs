mod config;
mod navigation;
mod device_stream;
mod sensor_data;
mod pos_sys;
mod sina;
mod preprocessing;

use std::env;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(version, about)]
#[command(group(
    clap::ArgGroup::new("mode")
        .required(true)
        .args(["nav", "preproc"]),
))]
struct Args {
    /// Path to configuration file
    #[arg(long, default_value = "config/default.toml")]
    config: String,

    /// Launch navigation mode with the input inside given folder.
    /// The folder must contain:
    /// semantic.txt (Scenescript output),
    ///  tags_world_config.json (can be computed using --preproc)
    #[arg(long)]
    nav: Option<String>,

    /// Launch preprocessing mode with the input inside the given folder.
    /// The folder must contain:
    /// video.mp4 (Extracted from .vrs),
    ///  closed_loop_trajectory.csv (Extracted from .vrs using MPS)
    #[arg(long)]
    preproc: Option<String>,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let config_path = args.config;
    let config = config::load_config(config_path)?;
    if let Some(nav_path) = args.nav {
        println!("Launching navigation mode: {}", nav_path);
        sina::Sina::new().launch(&config, nav_path, (3.0, 5.0))?;
    } else if let Some(preproc_path) = args.preproc {
        println!("Launching preprocessing: {}", preproc_path);
        preprocessing::preprocess(&config, preproc_path)?;
    }

    Ok(())
}
