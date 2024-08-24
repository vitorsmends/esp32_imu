#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.serial_port = '/dev/rfcomm0'
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate)
        self.get_logger().info("Connected to ESP32")

        # Initialize filter parameters
        self.alpha = 0.5  # Smoothing factor for the low-pass filter (0 < alpha <= 1)
        self.prev_ax = 0
        self.prev_ay = 0
        self.prev_az = 0
        self.prev_gx = 0
        self.prev_gy = 0
        self.prev_gz = 0

    def low_pass_filter(self, new_value, prev_value):
        return self.alpha * new_value + (1 - self.alpha) * prev_value

    def read_and_publish_imu_data(self):
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').rstrip()                
                if data.startswith("A"):
                    try:
                        ax, ay, az, gx, gy, gz = self.parse_imu_data(data)
                        
                        # Apply the low-pass filter to the data
                        ax_filtered = self.low_pass_filter(ax, self.prev_ax)
                        ay_filtered = self.low_pass_filter(ay, self.prev_ay)
                        az_filtered = self.low_pass_filter(az, self.prev_az)
                        gx_filtered = self.low_pass_filter(gx, self.prev_gx)
                        gy_filtered = self.low_pass_filter(gy, self.prev_gy)
                        gz_filtered = self.low_pass_filter(gz, self.prev_gz)

                        # Update the previous values with the filtered data
                        self.prev_ax, self.prev_ay, self.prev_az = ax_filtered, ay_filtered, az_filtered
                        self.prev_gx, self.prev_gy, self.prev_gz = gx_filtered, gy_filtered, gz_filtered
                        
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"
                        
                        imu_msg.linear_acceleration.x = ax_filtered * 9.81 / 16384.0
                        imu_msg.linear_acceleration.y = ay_filtered * 9.81 / 16384.0
                        imu_msg.linear_acceleration.z = az_filtered * 9.81 / 16384.0
                        
                        imu_msg.angular_velocity.x = gx_filtered * 3.14159 / 180.0 / 131.0
                        imu_msg.angular_velocity.y = gy_filtered * 3.14159 / 180.0 / 131.0
                        imu_msg.angular_velocity.z = gz_filtered * 3.14159 / 180.0 / 131.0
                        
                        self.publisher_.publish(imu_msg)
                    except ValueError as e:
                        self.get_logger().error(f"Failed to parse data: {e}")

            time.sleep(0.0167)

    def parse_imu_data(self, data):
        try:
            data = data[1:]  # Remove 'A'
            parts = data.split(',')
            ax = int(parts[0].strip())
            ay = int(parts[1].strip())
            az = int(parts[2].strip())
            gx = int(parts[3].replace('G', '').strip())  # Remove 'G' before converting
            gy = int(parts[4].strip())
            gz = int(parts[5].strip())
            return ax, ay, az, gx, gy, gz
        except Exception as e:
            self.get_logger().error(f"Error parsing IMU data: {e}")
            return 0, 0, 0, 0, 0, 0

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    
    try:
        imu_publisher.read_and_publish_imu_data()
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("Shutting down")
    finally:
        imu_publisher.ser.close()
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
