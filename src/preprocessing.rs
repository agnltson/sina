mod detector;
mod pose;
mod decoder;
mod worker;
mod config;

use std::fs;

pub fn load_frame_timestamps_us(path: &str) -> anyhow::Result<Vec<i64>> {
    let content = fs::read_to_string(path)?;
    let timestamps_ns: Vec<i64> = serde_json::from_str(&content)?;

    Ok(timestamps_ns.into_iter().map(|ns| ns / 1000).collect())
}
