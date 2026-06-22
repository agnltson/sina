use std::collections::BinaryHeap;

use crate::sensor_data::{ImuMessage, ImageMessage};

// TODO: handle and synchronize all sensor data
#[derive(Debug, Clone)]
pub struct SensorBuffer {
    imus: BinaryHeap<ImuMessage>,
    left_image: BinaryHeap<ImageMessage>,
    right_image: BinaryHeap<ImageMessage>,
}

impl SensorBuffer {
    pub fn new() -> Self {
        Self {
            imus: BinaryHeap::new(),
            left_image: BinaryHeap::new(),
            right_image: BinaryHeap::new(),
        }
    }

    pub fn push_imu(&mut self, imu: ImuMessage) {
        self.imus.push(imu);
    }

    pub fn push_image(&mut self, image: ImageMessage) {
        if image.camera == 0 {
            self.left_image.push(image);
        } else {
            self.right_image.push(image);
        }
    }

    pub fn drain_imu_until(&mut self, up_to_ns: u64) -> Vec<ImuMessage> {
        let mut result = Vec::new();
        while let Some(imu) = self.imus.peek() {
            if imu.timestamp_ns <= up_to_ns {
                result.push(self.imus.pop().unwrap());
            } else {
                break;
            }
        }
        result
    }

    pub fn pop_synced_image(&mut self) -> Option<(ImageMessage, ImageMessage)> {
        loop {
            let left = self.left_image.peek()?;
            let right = self.right_image.peek()?;

            let diff = (left.timestamp_ns as i64 - right.timestamp_ns as i64).unsigned_abs();

            if diff <= 5_000_000 {
                // Moins de 5ms d'écart : paire valide
                return Some((
                    self.left_image.pop().unwrap(),
                    self.right_image.pop().unwrap(),
                ));
            }

            if left.timestamp_ns < right.timestamp_ns {
                self.left_image.pop();
            } else {
                self.right_image.pop();
            }
        }
    }
}
