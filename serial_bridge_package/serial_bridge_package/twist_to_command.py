#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TwistToCommand(Node):
    def __init__(self):
        super().__init__('twist_to_command')
        
        # Подписка на топик Twist (управление движением)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Стандартный топик для управления движением
            self.twist_callback,
            10
        )
        
        # Публикатор в топик для отправки на ESP32
        self.publisher = self.create_publisher(
            String,
            '/esp32_input',  # Топик, из которого ваша нода берет данные для отправки на ESP32
            10
        )
        
        self.get_logger().info('Нода twist_to_command запущена. Ожидание команд Twist...')
    
    def twist_callback(self, msg):
        """Преобразование сообщения Twist в строку команды для ESP32"""
        # Извлекаем линейную и угловую скорости
        x_linear_velocity = msg.linear.x
        z_angular_velocity = msg.angular.z
        
        # Форматируем сообщение согласно протоколу
        command = f"${x_linear_velocity:.3f};{z_angular_velocity:.3f}#"
        
        # Создаем и публикуем сообщение String
        string_msg = String()
        string_msg.data = command
        
        self.publisher.publish(string_msg)
        self.get_logger().debug(f'Отправлена команда: {command}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TwistToCommand()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Нода twist_to_command остановлена')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()