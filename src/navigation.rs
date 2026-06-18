mod parser;
mod raw_data;
mod data;
mod room_graph;
mod room_topology;
mod room_cdt;
mod navmesh;
mod navgraph;
mod utils;
pub mod navigator;

pub use navigator::Navigator;

use nalgebra::Vector3;

pub enum VisualisationData {
    Position(Vector3<f64>, u64),
    LeftImage(Vec<u8>, u64),
    RightImage(Vec<u8>, u64),
}
