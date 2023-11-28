#!/usr/bin/env python3
# Driver: SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries

from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
import time
import board
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE
)

from adafruit_bno08x.i2c import BNO08X_I2C


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'bno08x/raw', 10)

    def publish(self, imu):
        self.publisher_.publish(imu)


class MagneticNode(Node):

    def __init__(self):
        super().__init__('magnetic_publisher')
        self.publisher_ = self.create_publisher(
            MagneticField, 'bno08x/mag', 10)

    def publish(self, magnetic):
        self.publisher_.publish(magnetic)


class DiagnosticNode(Node):

    def __init__(self):
        super().__init__('diagnostic')
        self.publisher_ = self.create_publisher(
            DiagnosticStatus, 'bno08x/status', 10)

    def publish(self, diagnostic):
        self.publisher_.publish(diagnostic)


def bno08x_node():
    # Initialize ROS node

    imu_node = ImuNode()
    magnetic_node = MagneticNode()
    diagnostic_node = DiagnosticNode()

    i2c = I2C(3)
    bno = BNO08X_I2C(i2c, address=0x4a)  # BNO080 (0x4b) BNO085 (0x4a)

    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)

    time.sleep(0.5)  # ensure IMU is initialized

    while True:
        raw_msg = Imu()

        accel_x, accel_y, accel_z = bno.acceleration
        raw_msg.linear_acceleration.x = accel_x
        raw_msg.linear_acceleration.y = accel_y
        raw_msg.linear_acceleration.z = accel_z

        gyro_x, gyro_y, gyro_z = bno.gyro
        raw_msg.angular_velocity.x = gyro_x
        raw_msg.angular_velocity.y = gyro_y
        raw_msg.angular_velocity.z = gyro_z

        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        raw_msg.orientation.w = quat_i
        raw_msg.orientation.x = quat_j
        raw_msg.orientation.y = quat_k
        raw_msg.orientation.z = quat_real

        raw_msg.orientation_covariance[0] = -1
        raw_msg.linear_acceleration_covariance[0] = -1
        raw_msg.angular_velocity_covariance[0] = -1

        imu_node.publish(raw_msg)

        mag_msg = MagneticField()
        mag_x, mag_y, mag_z = bno.magnetic
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z
        mag_msg.magnetic_field_covariance[0] = -1
        magnetic_node.publish(mag_msg)

        status_msg = DiagnosticStatus()
        status_msg.level = 0
        status_msg.name = "bno08x IMU"
        status_msg.message = ""
        diagnostic_node.publish(status_msg)

    rospy.loginfo(rospy.get_caller_id() + "  bno08x node finished")


def main():
    try:
        bno08x_node()
    except Exception:
        print("Error occured!")
