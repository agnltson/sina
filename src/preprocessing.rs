mod pose;
mod decoder;
mod worker;
mod config;

use std::{
    fs,
    thread,
};
use crossbeam_channel::bounded;

use crate::{
    config::Config,
    preprocessing::{
        worker::CameraParams,
    },
};

fn load_frame_timestamps_us(path: &str) -> anyhow::Result<Vec<i64>> {
    let content = fs::read_to_string(path)?;
    let timestamps_ns: Vec<i64> = serde_json::from_str(&content)?;

    Ok(timestamps_ns.into_iter().map(|ns| ns / 1000).collect())
}

const DEFAULT_NUM_WORKERS: usize = 4;
const DEFAULT_QUEUE_CAPACITY: usize = 32;
const DEFAULT_MAX_HAMMING: usize = 0;
const DEFAULT_MIN_OBSERVATION: usize = 5;

pub fn preprocess(config: &Config, data_path: String) -> anyhow::Result<()> {
    println!("Running preprocessing with {} input", data_path);
    let mp4_path = format!("{}/video.mp4", data_path);
    let trajectory_path = format!("{}/closed_loop_trajectory.csv", data_path);
    let timestamps_path = format!("{}/mp4_to_vrs_time_ns.csv", data_path);

    println!("Loading trajectory CSV...");
    let csv_poses = pose::load_poses(&trajectory_path)?;
    println!("  {} poses loaded.", csv_poses.len());

    println!("Loading timestamps...");
    let frame_timestamps_us = load_frame_timestamps_us(&timestamps_path)?;
    println!("  {} timestamps loaded.", frame_timestamps_us.len());

    let num_workers = config.preprocessor.num_workers.unwrap_or_else(|| {
        thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(DEFAULT_NUM_WORKERS)
    });
    println!("Launching pipeline with {num_workers} workers...");

    let queue_capacity = config.preprocessor.queue_capacity.unwrap_or(DEFAULT_QUEUE_CAPACITY);
    let (tx, rx) = bounded::<decoder::DecodedFrame>(queue_capacity);

    let skip_factor = config.preprocessor.skip_factor.unwrap_or(1);
    let decoder_handle = thread::spawn(move || decoder::decode_and_send(&mp4_path, skip_factor, tx));

    let camera = CameraParams {
        fx: config.preprocessor.fx,
        fy: config.preprocessor.fy,
        cx: config.preprocessor.cx,
        cy: config.preprocessor.cy,
        tag_size_m: config.apriltag.tag_size_m,
    };

    let mut worker_handles = Vec::with_capacity(num_workers);
    for worker_id in 0..num_workers {
        let rx = rx.clone();
        let csv_poses = csv_poses.clone();
        let frame_timestamps_us = frame_timestamps_us.clone();
        let tag_family_str = config.apriltag.tag_family.clone();
        let max_hamming = config.preprocessor.max_hamming.unwrap_or(DEFAULT_MAX_HAMMING);

        worker_handles.push(
            thread::Builder::new()
                .name(
                    format!("worker {}", worker_id).to_string())
                .spawn(move || {
                    worker::worker_loop(
                        rx,
                        &csv_poses,
                        &frame_timestamps_us,
                        camera,
                        &tag_family_str,
                        max_hamming,
                    )
                })?
        );
    }

    drop(rx);

    let _ = decoder_handle
        .join()
        .map_err(|_| anyhow::anyhow!("Decoding thread panicked"))?;

    let mut all_observations = Vec::new();
    for (worker_id, handle) in worker_handles.into_iter().enumerate() {
        let observations = handle
            .join()
            .map_err(|_| anyhow::anyhow!("worker {worker_id} panicked"))??;
        all_observations.extend(observations);
    }

    println!(
        "{} tags observed, aggregation...",
        all_observations.len()
    );
    let min_observation = config.preprocessor.min_observation.unwrap_or(DEFAULT_MIN_OBSERVATION);
    config::write_config(&all_observations, "tags_world_config.json", min_observation)?;
    println!("AprilTag config written in tags_world_config.json");

    Ok(())
}
