#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
ROS2-мост для двунаправленной связи между UART-устройствами и ROS-топиками с
буферизацией и парсингом сообщений в формате $message#. Использует
многопоточность для асинхронного чтения порта, поддерживает настраиваемые
параметры подключения. Требует прав доступа к последовательному порту и
стабильного соединения с устройством.

ANNOTATION
ROS2 bridge for bidirectional communication between UART devices and ROS
topics with message buffering and parsing in $message# format. Uses
multithreading for asynchronous port reading, supports configurable
connection parameters. Requires serial port access privileges and stable
device connection.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading


class SerialBridgeNode(Node):
    """
    ROS2 node for bridging between ROS2 topics and serial port communication.
    """

    def __init__(self):
        super().__init__('serial_bridge_node')

        # Declare parameters for port configuration
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ros_out_topic', 'serial/from')
        self.declare_parameter('ros_in_topic', 'serial/to')

        # Get parameter values
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        ros_out_topic = self.get_parameter('ros_out_topic').get_parameter_value().string_value
        ros_in_topic = self.get_parameter('ros_in_topic').get_parameter_value().string_value

        # Initialize serial port
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=0.5  # Read timeout in seconds
            )
            self.get_logger().info(
                f"Serial port {serial_port} opened at {baudrate} baud"
                )
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening port {serial_port}: {e}")
            raise e

        # Create Publisher for sending data FROM serial port TO ROS
        self.pub_to_ros = self.create_publisher(String, ros_out_topic, 10)

        # Create Subscriber for receiving data FROM ROS and sending TO serial port
        self.sub_from_ros = self.create_subscription(
            String,
            ros_in_topic,
            self.ros_callback,
            10
        )

        # Flag for controlling worker threads
        self.running = True

        # Start thread for reading from serial port
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True  # Thread will exit when main thread exits
        self.read_thread.start()

        self.get_logger().info("Serial bridge node started. Waiting for data...")

    def ros_callback(self, msg):
        """
        Callback for messages received from ROS topic. Sends data to
        serial port.
        """
        try:
            data_to_send = msg.data
            self.ser.write(data_to_send.encode('utf-8'))
            self.get_logger().debug(f"Sent to UART: {data_to_send.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def read_from_serial(self):
        """
        Worker thread: reads data from serial port and publishes to ROS topic.
        """
        # Buffer for accumulating data between reads
        if not hasattr(self, 'serial_buffer'):
            self.serial_buffer = ''
        
        while self.running and rclpy.ok():
            try:
                # Read all available data or wait for timeout
                if self.ser.in_waiting > 0:
                    data_bytes = self.ser.read(self.ser.in_waiting)

                    try:
                        data_str = data_bytes.decode('utf-8', errors='ignore')
                        self.serial_buffer += data_str
                        
                        # Process all complete messages in buffer
                        while True:
                            # Find message start
                            start_index = self.serial_buffer.find('$')
                            if start_index == -1:
                                # No message start found, clear buffer
                                self.serial_buffer = ''
                                break
                            
                            # Find message end after start
                            end_index = self.serial_buffer.find('#', start_index + 1)
                            if end_index == -1:
                                # Message not complete, keep everything after last $ in buffer
                                self.serial_buffer = self.serial_buffer[start_index:]
                                break
                            
                            # Extract complete message
                            full_message = self.serial_buffer[start_index:end_index + 1]
                            
                            # Validate message format (must start with $ and end with #)
                            if full_message.startswith('$') and full_message.endswith('#'):
                                message_text = full_message.strip()
                                
                                # Publish extracted text
                                ros_msg = String()
                                ros_msg.data = message_text
                                self.pub_to_ros.publish(ros_msg)
                                self.get_logger().debug(
                                    f"Received complete message: {full_message} -> {message_text}"
                                )
                            else:
                                self.get_logger().warning(
                                    f"Invalid message format: {full_message}"
                                )
                            
                            # Remove processed message from buffer
                            remaining_data = self.serial_buffer[end_index + 1:]
                            self.serial_buffer = remaining_data
                            
                            # Exit loop if no data left in buffer
                            if not self.serial_buffer:
                                break

                    except UnicodeDecodeError:
                        self.get_logger().warning(
                            "Failed to decode data as UTF-8"
                            )

            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {e}")
                break

    def destroy_node(self):
        """Cleanup when node is stopped."""
        self.get_logger().info("Shutting down serial bridge node...")
        self.running = False
        
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        
        super().destroy_node()


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    serial_bridge_node = SerialBridgeNode()

    try:
        rclpy.spin(serial_bridge_node)
    except KeyboardInterrupt:
        serial_bridge_node.get_logger().info(
            "Serial bridge-node stopped by user (Ctrl+C)"
            )
    finally:
        serial_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()