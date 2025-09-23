from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Нода преобразования Twist в команды для ESP32
    twist_to_command = Node(
        package='serial_bridge_package',
        executable='twist_to_command',
        name='twist_to_command',
        namespace='low_level',
        output='screen',
        remappings=[
            ("/esp32_input", "serial/cmd"),
            ("/cmd_vel", "/cmd_vel"),
        ],
    )

    bridge_node = Node(
        package='serial_bridge_package',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        namespace='low_level',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'timeout': 1,
            'ros_in_topic': 'serial/cmd',
            'ros_out_topic': 'serial/feedback',
        }],
    )
    
    # Нода обработки обратной связи от ESP32
    feedback_node = Node(
        package='serial_bridge_package',
        executable='feedback_processor',
        name='feedback_processor',
        namespace='low_level',
        parameters=[{
            'frame_id': 'odom',
            'child_frame_id': 'base_link'
        }],
        output='screen',
        remappings=[
            ("/esp32_feedback", "serial/feedback"),
        ],
    )

    return LaunchDescription([
        twist_to_command,
        bridge_node,
        feedback_node

    ])