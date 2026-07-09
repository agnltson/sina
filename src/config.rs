use std::fs;
use std::path::Path;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct Config {
    pub preprocessor: PreprocessorConfig,
    pub streaming: StreamingConfig,
    pub apritag: ApriltagConfig,
}

#[derive(Debug, Deserialize)]
pub struct ApriltagConfig {
    pub tag_family: String,
    pub tag_size_m: f64,
}

#[derive(Debug, Deserialize)]
pub struct StreamingConfig {
    pub profile: String,
    pub ip: String,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

#[derive(Debug, Deserialize)]
pub struct PreprocessorConfig {
    pub skip_factor: Option<usize>,
    pub num_workers: Option<usize>,
    pub queue_capacity: Option<usize>,
    pub min_observation: Option<usize>,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

pub fn load_config<P: AsRef<Path>>(path: P) -> Result<Config, Box<dyn std::error::Error>> {
    let contents = fs::read_to_string(path)?;
    let config: Config = toml::from_str(&contents)?;
    Ok(config)
}
