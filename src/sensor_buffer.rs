use std::collections::BinaryHeap;

use crate::sensor_data::{ImuMessage, ImageMessage};

// TODO: handle and synchronize all sensor data
#[derive(Debug, Clone)]
pub struct SensorBuffer {
    imu_heap: BinaryHeap<ImuMessage>,
    image: Option<ImageMessage>,
}

impl SensorBuffer {
    pub fn new() -> Self {
        Self {
            imu_heap: BinaryHeap::new(),
            image: None,
        }
    }

    pub fn push_imu(&mut self, imu: ImuMessage) {
        self.imu_heap.push(imu);
    }

    pub fn push_image(&mut self, image: ImageMessage) {
        self.image = Some(image);
    }

    pub fn pop_imu(&mut self) -> Option<ImuMessage> {
        self.imu_heap.pop()
    }

    pub fn get_image(&self) -> Option<ImageMessage> {
        self.image.clone()
    }

    pub fn is_ready_to_process(&self) -> bool {
        !self.image.is_none()
    }

    pub fn clear(&mut self) {
        self.imu_heap.clear();
        self.image = None;
    }
}
