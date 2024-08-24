#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import numpy as np

class OrientationEstimator(Node):
    def __init__(self):
        super().__init__('orientation_estimator')
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.orientation_publisher = self.create_publisher(Float32MultiArray, '/estimated_orientation', 10)

    def imu_callback(self, msg: Imu):
        # Estimate orientation in Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        orientation_msg = Float32MultiArray()
        orientation_msg.data = [roll, pitch, yaw]
        self.orientation_publisher.publish(orientation_msg)

    def quaternion_to_euler(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.sign(sinp) * np.pi / 2

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    orientation_estimator = OrientationEstimator()
    
    rclpy.spin(orientation_estimator)

    orientation_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
