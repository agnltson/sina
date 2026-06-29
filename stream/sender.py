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

    def on_pose_received(self, pose):
        """
        pose.world_T_device = SE3 (world -> device)
        """

        T = pose.world_T_device

        if self.T0 is None:
            self.T0 = T
            return

        T_rel = T @ self.T0.inverse()

        self.send({
            "type": "pose_relative",
            "timestamp_ns": pose.timestamp_ns,

            "position_m": [
                T_rel.translation[0],
                T_rel.translation[1],
                T_rel.translation[2],
            ],

            "rotation": T_rel.euler_angles().tolist(),
        })

    def on_image_received(self, image, record):
        pass

    def on_imu_received(self, samples, imu_idx):
        pass

    def on_magneto_received(self, sample) -> None:
        pass

    def on_baro_received(self, sample) -> None:
        pass

    def on_streaming_client_failure(self, reason, message: str) -> None:
        pass
