#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TwistToCommand(Node):
    """Node for converting Twist messages to custom ESP32 command protocol."""

    def __init__(self):
        super().__init__('twist_to_command')
        
        # Subscription to Twist topic (movement control)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Standard topic for movement control
            self.twist_callback,
            10
        )
        
        # Publisher for ESP32 communication topic
        self.publisher = self.create_publisher(
            String,
            '/esp32_input',  # Topic used for sending data to ESP32
            10
        )
        
        self.get_logger().info(
            'Twist-to-command node started. Waiting for Twist commands...'
        )

    def twist_callback(self, msg):
        """Convert Twist message to ESP32 command string."""
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Format message according to protocol
        command = f"${linear_x:.3f};{angular_z:.3f}#"
        
        # Create and publish String message
        string_msg = String()
        string_msg.data = command
        
        self.publisher.publish(string_msg)
        self.get_logger().debug(f'Sent command: {command}')


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = TwistToCommand()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Twist-to-command node stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()