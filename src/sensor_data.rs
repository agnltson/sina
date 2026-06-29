use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct RawStateMessage {
    #[serde(rename ="type")]
    pub msg_type: String,
    pub position_m: [f64; 3],
    pub rotation: [f64; 3],
}

#[derive(Debug, Deserialize)]
pub struct StateMessage {
    pub position: [f64; 3],
    pub rotation: [f64; 3],
}

impl StateMessage {
    pub fn from_json(raw_str: &String) -> anyhow::Result<Self> {
        let p: RawStateMessage = serde_json::from_str(&raw_str)?;
        Ok( Self {
            position: p.position_m,
            rotation: p.rotation,
        })
    }
}
