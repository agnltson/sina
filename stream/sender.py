from collections import deque
from typing import Sequence

import aria.sdk as aria
from common import ctrl_c_handler

from projectaria_tools.core.sensor_data import (
    ImageDataRecord,
    MotionData,
)

from projectaria_tools.core.calibration import (
    distort_by_calibration,
)

import zmq
import json
import cv2
import base64

class ZMQDataSender:
    def __init__(
        self,
        slam1_calib,
        slam2_calib,
        dst_slam1,
        dst_slam2,
        endpoint="tcp://*:5555",
    ):
        self.slam1_calib = slam1_calib
        self.slam2_calib = slam2_calib
        self.dst_slam1 = dst_slam1
        self.dst_slam2 = dst_slam2

        ctx = zmq.Context.instance()
        self.socket = ctx.socket(zmq.PUB)

        self.socket.setsockopt(zmq.SNDHWM, 1000)
        self.socket.setsockopt(zmq.LINGER, 0)

        self.socket.bind(endpoint)

    def send(self, msg: dict):
        self.socket.send_string(json.dumps(msg))

    def on_image_received(self, image, record):

        if record.camera_id == aria.CameraId.Slam1:
            src_calib = self.slam1_calib
            dst_calib = self.dst_slam1

        elif record.camera_id == aria.CameraId.Slam2:
            src_calib = self.slam2_calib
            dst_calib = self.dst_slam2

        else:
            return

        pinhole_image = distort_by_calibration(
            image,
            dst_calib,
            src_calib,
        )

        ok, encoded = cv2.imencode(".jpg", pinhole_image)
        if not ok:
            return

        self.send({
            "type": "slam_image",
            "camera": str(record.camera_id),
            "timestamp_ns": record.capture_timestamp_ns,
            "jpeg": base64.b64encode(encoded.tobytes()).decode("ascii"),
        })

    def on_imu_received(self, samples, imu_idx):
        s = samples[0]

        self.send({
            "type": "imu",
            "imu_idx": imu_idx,
            "timestamp_ns": s.capture_timestamp_ns,
            "accel_msec2": list(s.accel_msec2),
            "gyro_radsec": list(s.gyro_radsec),
        })

    def on_magneto_received(self, sample) -> None:
        pass

    def on_baro_received(self, sample) -> None:
        pass

    def on_streaming_client_failure(self, reason, message: str) -> None:
        pass
