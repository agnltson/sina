use serde::Serialize;
use std::collections::HashMap;
use std::fs;

use super::worker::TagObservation;

#[derive(Debug, Serialize)]
pub struct TagConfigEntry {
    pub id: usize,
    pub position_world: [f64; 3],
    pub orientation_world_wxyz: [f64; 4],
    pub num_observations: usize,
    pub position_std_m: [f64; 3],
    pub best_reprojection_error: f64,
}

#[derive(Debug, Serialize)]
pub struct TagsConfig {
    pub tags: Vec<TagConfigEntry>,
}

pub fn write_config(
    observations: &[TagObservation],
    output_path: &str,
    min_observations: usize,
) -> anyhow::Result<()> {
    let mut by_tag: HashMap<usize, Vec<&TagObservation>> = HashMap::new();
    for obs in observations {
        by_tag.entry(obs.tag_id).or_default().push(obs);
    }

    let mut tags: Vec<TagConfigEntry> = Vec::new();
    for (tag_id, obs_list) in by_tag {
        if obs_list.len() < min_observations {
            continue;
        }

        let n = obs_list.len() as f64;
        let mean = obs_list.iter().fold([0.0; 3], |acc, o| {
            [
                acc[0] + o.translation_world[0] / n,
                acc[1] + o.translation_world[1] / n,
                acc[2] + o.translation_world[2] / n,
            ]
        });

        let variance = obs_list.iter().fold([0.0; 3], |acc, o| {
            [
                acc[0] + (o.translation_world[0] - mean[0]).powi(2) / n,
                acc[1] + (o.translation_world[1] - mean[1]).powi(2) / n,
                acc[2] + (o.translation_world[2] - mean[2]).powi(2) / n,
            ]
        });
        let std_dev = [
            variance[0].sqrt(),
            variance[1].sqrt(),
            variance[2].sqrt(),
        ];

        let best = obs_list
            .iter()
            .min_by(|a, b| {
                a.reprojection_error
                    .partial_cmp(&b.reprojection_error)
                    .unwrap()
            })
            .expect("obs_list non empty (verified by min_observations >= 1)");

        tags.push(TagConfigEntry {
            id: tag_id,
            position_world: mean,
            orientation_world_wxyz: best.quaternion_world_wxyz,
            num_observations: obs_list.len(),
            position_std_m: std_dev,
            best_reprojection_error: best.reprojection_error,
        });
    }

    tags.sort_by_key(|t| t.id);

    let config = TagsConfig { tags };
    let json = serde_json::to_string_pretty(&config)?;
    fs::write(output_path, json)?;

    Ok(())
}
