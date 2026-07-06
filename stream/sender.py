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

import numpy as np
import zmq
import json
import cv2
import base64
import time
import threading

class ZMQDataSender:
    def __init__(self, rgb_calib, dst_rgb, endpoint="tcp://*:5555"):
        self.rgb_calib = rgb_calib
        self.dst_rgb = dst_rgb
        self._latest_image = None
        self._latest_record = None
        self._state_lock = threading.Lock()
        self._socket_lock = threading.Lock()

        ctx = zmq.Context.instance()
        self.socket = ctx.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1000)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(endpoint)

    def send(self, msg: dict):
        with self._socket_lock:
            self.socket.send_string(json.dumps(msg))

    def on_magneto_received(self, sample: MotionData) -> None:
        self.send({
            "type": "mag",
            "timestamp_ns": sample.capture_timestamp_ns,
            "mag_tesla": list(sample.mag_tesla),
        })
        pass

    def on_image_received(self, image, record):
        if int(record.camera_id) != int(aria.CameraId.Rgb):
            return
        with self._state_lock:
            self._latest_image = image.copy()
            self._latest_record = record

    def process_loop(self):
        while True:
            with self._state_lock:
                image = self._latest_image
                record = self._latest_record
                self._latest_image = None
            if image is None:
                time.sleep(0.001)
                continue

            rgb_image = cv2.cvtColor(np.rot90(image, -1), cv2.COLOR_BGR2RGB)
            ok, encoded = cv2.imencode(".jpg", rgb_image)
            if not ok:
                continue
            self.send({
                "type": "rgb_image",
                "camera": "rgb",
                "timestamp_ns": record.capture_timestamp_ns,
                "jpeg": base64.b64encode(encoded.tobytes()).decode("ascii"),
            })
