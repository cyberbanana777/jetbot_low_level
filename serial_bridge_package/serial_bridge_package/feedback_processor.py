#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS2-узел для обработки сырых данных с ESP32 в формате $data1;...;data13# с
публикацией в темы одометрии, IMU, температуры, напряжения, нагрузки и позиции
колес. Преобразует углы в кватернионы и публикует преобразования систем
координат через TF. Ожидает строго форматированные сообщения с 13 числовыми
полями без обработки ошибок связи.

ANNOTATION
ROS2 node for processing raw ESP32 data in $data1;...;data13# format with
publishing to odometry, IMU, temperature, voltage, load and wheel position
topics. Converts angles to quaternions and broadcasts coordinate frame
transformations via TF. Requires strictly formatted 13-field numeric messages
without communication error handling.
'''

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class FeedbackProcessor(Node):
    """
    ROS2 node for processing feedback data from ESP32 and publishing to 
    various topics.
    """

    def __init__(self):
        super().__init__('feedback_processor')
        
        # Subscription to raw data from ESP32
        self.subscription = self.create_subscription(
            String,
            '/esp32_feedback',  # Topic where ESP32 data is published
            self.feedback_callback,
            10
        )
        
        # Publishers for various data types
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.left_temp_pub = self.create_publisher(
            Temperature, '/sensors/wheel/left/temperature', 10
            )
        self.right_temp_pub = self.create_publisher(
            Temperature, '/sensors/wheel/right/temperature', 10
            )
        self.left_voltage_pub = self.create_publisher(
            Float32, '/sensors/wheel/left/voltage', 10
            )
        self.right_voltage_pub = self.create_publisher(
            Float32, '/sensors/wheel/right/voltage', 10
            )
        self.left_load_pub = self.create_publisher(
            Float32, '/sensors/wheel/left/load', 10
            )
        self.right_load_pub = self.create_publisher(
            Float32, '/sensors/wheel/right/load', 10
            )
        self.left_position_pub = self.create_publisher(
            Float32, '/sensors/wheel/left/position', 10
            )
        self.right_position_pub = self.create_publisher(
            Float32, '/sensors/wheel/right/position', 10
            )

        # TF broadcaster for coordinate system transformations
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Parameters for coordinate frames
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('left_wheel_frame_id', 'left_wheel')
        self.declare_parameter('right_wheel_frame_id', 'right_wheel')

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.left_wheel_frame_id = self.get_parameter('left_wheel_frame_id').value
        self.right_wheel_frame_id = self.get_parameter('right_wheel_frame_id').value

        
        self.get_logger().info(
            'Feedback processor node started. Waiting for ESP32 data...'
            )

    def feedback_callback(self, msg):
        """
        Process raw message from ESP32 and publish to appropriate topics.
        """
        try:
            raw_data = msg.data.strip()
            
            # Validate message format
            if not raw_data.startswith('$') or not raw_data.endswith('#'):
                self.get_logger().warning(
                    f'Invalid message format: {raw_data}'
                    )
                return
            
            # Remove starting $ and ending #
            content = raw_data[1:-1]
            
            # Split data by semicolons
            parts = content.split(';')
            
            if len(parts) >= 13:  # Check if all fields are present
                # Parse data
                x_position = float(parts[0])
                y_position = float(parts[1])
                omega_angle = float(parts[2])
                x_real_linear_velocity = float(parts[3])
                z_real_angular_velocity = float(parts[4])
                left_wheel_position = float(parts[5])
                right_wheel_position = float(parts[6])
                left_wheel_load = float(parts[7])
                right_wheel_load = float(parts[8])
                left_wheel_temperature = float(parts[9])
                right_wheel_temperature = float(parts[10])
                left_wheel_voltage = float(parts[11])
                right_wheel_voltage = float(parts[12])
                
                # Publish data to appropriate topics
                self.publish_odometry(
                    x_position, y_position, omega_angle,
                    x_real_linear_velocity, z_real_angular_velocity
                )
                self.publish_imu(omega_angle, z_real_angular_velocity)
                self.publish_temperature(
                    left_wheel_temperature,
                    right_wheel_temperature
                    )
                self.publish_voltage(left_wheel_voltage, right_wheel_voltage)
                self.publish_load(left_wheel_load, right_wheel_load)
                self.publish_position(
                    left_wheel_position,
                    right_wheel_position
                    )
                
                self.get_logger().debug(
                    f'Processed ESP32 message: position ({x_position:.2f}, {y_position:.2f})'
                )
                
            else:
                self.get_logger().warning(
                    f'Insufficient data in message. Expected 13 fields, got {len(parts)}'
                )
                
        except (ValueError, IndexError) as e:
            self.get_logger().warning(
                f'Error parsing message: {msg.data}. Error: {e}'
                )

    def publish_odometry(self, x, y, theta, vx, vth):
        """Publish odometry data."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        
        # Position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert angle to quaternion)
        q = self.angle_to_quaternion(theta)
        odom_msg.pose.pose.orientation = q
        
        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transformation
        self.publish_tf_transform(x, y, theta)

    def publish_tf_transform(self, x, y, theta):
        """Publish transformation between odom and base_link frames."""
        transform = TransformStamped()
        
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        
        q = self.angle_to_quaternion(theta)
        transform.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(transform)

    def publish_imu(self, theta, vth):
        """Publish IMU data (simplified - only angle and angular velocity)."""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Orientation
        q = self.angle_to_quaternion(theta)
        imu_msg.orientation = q
        
        # Angular velocity
        imu_msg.angular_velocity.z = vth
        
        self.imu_pub.publish(imu_msg)

    def publish_temperature(self, left_temp, right_temp):
        """Publish motor temperatures."""
        left_temp_msg = Temperature()
        left_temp_msg.header.stamp = self.get_clock().now().to_msg()
        left_temp_msg.header.frame_id = self.left_wheel_frame_id
        left_temp_msg.temperature = left_temp
        self.left_temp_pub.publish(left_temp_msg)
        
        right_temp_msg = Temperature()
        left_temp_msg.header.frame_id = self.right_wheel_frame_id
        right_temp_msg.header.stamp = self.get_clock().now().to_msg()
        right_temp_msg.temperature = right_temp
        self.right_temp_pub.publish(right_temp_msg)

    def publish_voltage(self, left_voltage, right_voltage):
        """Publish motor voltages."""
        left_voltage_msg = Float32()
        left_voltage_msg.data = left_voltage
        self.left_voltage_pub.publish(left_voltage_msg)
        
        right_voltage_msg = Float32()
        right_voltage_msg.data = right_voltage
        self.right_voltage_pub.publish(right_voltage_msg)

    def publish_load(self, left_load, right_load):
        """Publish motor loads."""
        left_load_msg = Float32()
        left_load_msg.data = left_load
        self.left_load_pub.publish(left_load_msg)
        
        right_load_msg = Float32()
        right_load_msg.data = right_load
        self.right_load_pub.publish(right_load_msg)

    def publish_position(self, left_position, right_position):
        """Publish motor shaft positions."""
        left_position_msg = Float32()
        left_position_msg.data = left_position
        self.left_position_pub.publish(left_position_msg)
        
        right_position_msg = Float32()
        right_position_msg.data = right_position
        self.right_position_pub.publish(right_position_msg)

    @staticmethod
    def angle_to_quaternion(angle):
        """Convert angle (in radians) to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(angle / 2.0)
        q.w = math.cos(angle / 2.0)
        return q


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = FeedbackProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()