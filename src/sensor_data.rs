use serde::Deserialize;
use base64::Engine;
use nalgebra::{Vector3, Quaternion};

use std::cmp::Ordering;

#[derive(Debug)]
pub enum SensorData {
    Imu(ImuMessage),
    Image(ImageMessage),
}

#[derive(Debug, Deserialize)]
pub struct RawImuMessage {
    #[serde(rename = "type")]
    pub msg_type: String,
    pub imu_idx: u32,
    pub timestamp_ns: u64,
    pub accel_msec2: [f64; 3],
    pub gyro_radsec: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct ImuMessage {
    pub imu_idx: u32,
    pub timestamp_ns: u64,
    pub accel_msec2: Vector3<f64>,
    pub gyro_radsec: Vector3<f64>,
}

impl Eq for ImuMessage {}

impl PartialEq for ImuMessage {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp_ns.eq(&other.timestamp_ns)
    }
}

impl Ord for ImuMessage {
    fn cmp(&self, other: &Self) -> Ordering {
        self.timestamp_ns.cmp(&other.timestamp_ns).reverse()
    }
}

impl PartialOrd for ImuMessage {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl ImuMessage {
    pub fn from_json(raw_str: &String) -> anyhow::Result<Self> {
        let i: RawImuMessage = serde_json::from_str(&raw_str)?;
        Ok(Self {
            imu_idx: i.imu_idx,
            timestamp_ns: i.timestamp_ns,
            accel_msec2: Vector3::new(i.accel_msec2[0], i.accel_msec2[1], i.accel_msec2[2]),
            gyro_radsec: Vector3::new(i.gyro_radsec[0], i.gyro_radsec[1], i.gyro_radsec[2]),
        }
            )
    }
}

#[derive(Debug, Deserialize)]
pub struct RawImageMessage {
    #[serde(rename = "type")]
    pub msg_type: String,
    pub camera: String,
    pub timestamp_ns: u64,
    // base64-encoded JPEG string from Python
    pub jpeg: String,
}

#[derive(Debug, Clone)]
pub struct ImageMessage {
    pub camera: u8,
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
            camera: raw.camera.as_str().parse::<u8>()?,
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
