import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import numpy as np
import cv2

from projectaria_tools.core.calibration import distort_by_calibration
from projectaria_tools.core.sensor_data import ImageDataRecord, MotionData
import aria.sdk as aria

class ROS2DataSender(Node):
    def __init__(self, slam1_calib, slam2_calib, dst_slam1, dst_slam2):
        super().__init__('aria_publisher')

        self.slam1_calib = slam1_calib
        self.slam2_calib = slam2_calib
        self.dst_slam1 = dst_slam1
        self.dst_slam2 = dst_slam2
        self.bridge = CvBridge()

        self.pub_left  = self.create_publisher(Image, '/aria/camera/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/aria/camera/right/image_raw', 10)
        self.pub_imu   = self.create_publisher(Imu, '/aria/imu0', 10)

    def on_image_received(self, image, record):
        if record.camera_id == aria.CameraId.Slam1:
            src_calib = self.slam1_calib
            dst_calib = self.dst_slam1
            pub = self.pub_left
        elif record.camera_id == aria.CameraId.Slam2:
            src_calib = self.slam2_calib
            dst_calib = self.dst_slam2
            pub = self.pub_right
        else:
            return

        pinhole_image = distort_by_calibration(image, dst_calib, src_calib)

        if len(pinhole_image.shape) == 2:
            ros_img = self.bridge.cv2_to_imgmsg(pinhole_image, encoding='mono8')
        else:
            ros_img = self.bridge.cv2_to_imgmsg(pinhole_image, encoding='bgr8')

        ts_ns = record.capture_timestamp_ns
        ros_img.header.stamp.sec     = ts_ns // 1_000_000_000
        ros_img.header.stamp.nanosec = ts_ns %  1_000_000_000
        ros_img.header.frame_id = 'camera'

        pub.publish(ros_img)

    def on_imu_received(self, samples, imu_idx):
        if imu_idx != 1: # only left imu
            return

        s = samples[0]
        msg = Imu()

        ts_ns = s.capture_timestamp_ns
        msg.header.stamp.sec     = ts_ns // 1_000_000_000
        msg.header.stamp.nanosec = ts_ns %  1_000_000_000
        msg.header.frame_id = 'imu'

        msg.linear_acceleration.x = s.accel_msec2[0]
        msg.linear_acceleration.y = s.accel_msec2[1]
        msg.linear_acceleration.z = s.accel_msec2[2]

        msg.angular_velocity.x = s.gyro_radsec[0]
        msg.angular_velocity.y = s.gyro_radsec[1]
        msg.angular_velocity.z = s.gyro_radsec[2]

        msg.orientation_covariance[0] = -1.0

        self.pub_imu.publish(msg)

    def on_magneto_received(self, sample): pass
    def on_baro_received(self, sample): pass
    def on_streaming_client_failure(self, reason, message): pass
