use serde::Deserialize;
use base64::Engine;
use nalgebra::Vector3;

use std::cmp::Ordering;

pub enum SensorData {
    Mag(MagMessage),
    Image(ImageMessage),
}

#[derive(Debug, Deserialize)]
pub struct RawMagMessage {
    #[serde(rename = "type")]
    pub _msg_type: String,
    pub timestamp_ns: u64,
    pub mag_tesla: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct MagMessage {
    pub timestamp_ns: u64,
    pub mag_tesla: Vector3<f64>,
}

impl Eq for MagMessage {}

impl PartialEq for MagMessage {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp_ns.eq(&other.timestamp_ns)
    }
}

impl Ord for MagMessage {
    fn cmp(&self, other: &Self) -> Ordering {
        self.timestamp_ns.cmp(&other.timestamp_ns).reverse()
    }
}

impl PartialOrd for MagMessage {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl MagMessage {
    pub fn from_json(raw_str: &String) -> anyhow::Result<Self> {
        let i: RawMagMessage = serde_json::from_str(&raw_str)?;
        Ok(Self {
            timestamp_ns: i.timestamp_ns,
            mag_tesla: Vector3::new(i.mag_tesla[0], i.mag_tesla[1], i.mag_tesla[2]),
        }
            )
    }
}

#[derive(Debug, Deserialize)]
pub struct RawImageMessage {
    #[serde(rename = "type")]
    pub _msg_type: String,
    #[serde(rename = "camera")]
    pub _camera: String,
    pub timestamp_ns: u64,
    pub jpeg: String,
}

#[derive(Debug, Clone)]
pub struct ImageMessage {
    pub timestamp_ns: u64,
    pub jpeg: Vec<u8>,
}

impl Eq for ImageMessage {}

impl PartialEq for ImageMessage {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp_ns.eq(&other.timestamp_ns)
    }
}

impl Ord for ImageMessage {
    fn cmp(&self, other: &Self) -> Ordering {
        self.timestamp_ns.cmp(&other.timestamp_ns).reverse()
    }
}

impl PartialOrd for ImageMessage {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl ImageMessage {
    pub fn from_json(raw_str: &String) -> anyhow::Result<Self> {
        let raw: RawImageMessage = serde_json::from_str(&raw_str)?;
        let image: Vec<u8> = decode_jpeg(&raw.jpeg)?;
        Ok(Self {
            timestamp_ns: raw.timestamp_ns,
            jpeg: image,
        })
    }
}

fn decode_jpeg(msg: &String) -> anyhow::Result<Vec<u8>> {
    let bytes = base64::engine::general_purpose::STANDARD
        .decode(&msg)?;

    Ok(bytes)
}
