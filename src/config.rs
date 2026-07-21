use std::fs;
use std::path::Path;
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    pub streaming: StreamingConfig,
    pub apriltag: ApriltagConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ApriltagConfig {
    pub tag_family: String,
    pub tag_size_m: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct StreamingConfig {
    pub profile: String,
    pub ip: String,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

pub fn load_config<P: AsRef<Path>>(path: P) -> anyhow::Result<Config> {
    let contents = fs::read_to_string(path)?;
    let config: Config = toml::from_str(&contents)?;
    Ok(config)
}
