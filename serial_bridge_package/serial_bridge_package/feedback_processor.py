#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class FeedbackProcessor(Node):
    def __init__(self):
        super().__init__('feedback_processor')
        
        # Подписка на топик с сырыми данными от ESP32
        self.subscription = self.create_subscription(
            String,
            '/esp32_feedback',  # Топик, в который ваша нода публикует данные от ESP32
            self.feedback_callback,
            10
        )
        
        # Публикаторы для различных типов данных
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.left_temp_pub = self.create_publisher(Temperature, '/sensors/wheel/left/temperature', 10)
        self.right_temp_pub = self.create_publisher(Temperature, '/sensors/wheel/right/temperature', 10)
        self.left_voltage_pub = self.create_publisher(Float32, '/sensors/wheel/left/voltage', 10)
        self.right_voltage_pub = self.create_publisher(Float32, '/sensors/wheel/right/voltage', 10)
        self.left_load_pub = self.create_publisher(Float32, '/sensors/wheel/left/load', 10)
        self.right_load_pub = self.create_publisher(Float32, '/sensors/wheel/right/load', 10)
        self.left_position_pub = self.create_publisher(Float32, '/sensors/wheel/left/position', 10)
        self.right_position_pub = self.create_publisher(Float32, '/sensors/wheel/right/position', 10)

        
        # TF broadcaster для трансформации между системами координат
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Параметры для систем координат
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        self.get_logger().info('Нода feedback_processor запущена. Ожидание данных от ESP32...')
    
    def feedback_callback(self, msg):
        """Обработка сырого сообщения от ESP32 и публикация в соответствующие топики"""
        try:
            # Получаем строку из сообщения
            raw_data = msg.data.strip()
            
            # Проверяем, что сообщение имеет правильный формат
            if not raw_data.startswith('$') or not raw_data.endswith('#'):
                self.get_logger().warning(f'Некорректный формат сообщения: {raw_data}')
                return
            
            # Удаляем начальный $ и конечный #
            content = raw_data[1:-1]
            
            # Разделяем данные по точкам с запятой
            parts = content.split(';')
            
            if len(parts) >= 13:  # Проверяем, что все поля присутствуют
                # Парсим данные
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
                
                # Публикуем данные в соответствующие топики
                self.publish_odometry(x_position, y_position, omega_angle, 
                                    x_real_linear_velocity, z_real_angular_velocity)
                
                self.publish_imu(omega_angle, z_real_angular_velocity)
                
                self.publish_temperature(left_wheel_temperature, right_wheel_temperature)
                
                self.publish_voltage(left_wheel_voltage, right_wheel_voltage)

                self.publish_load(left_wheel_load, right_wheel_load)

                self.publish_position(left_wheel_position, right_wheel_position)
                
                self.get_logger().debug(f'Обработано сообщение от ESP32: позиция ({x_position:.2f}, {y_position:.2f})')
                
            else:
                self.get_logger().warning(f'Недостаточно данных в сообщении. Ожидалось 13 полей, получено {len(parts)}')
                
        except (ValueError, IndexError) as e:
            self.get_logger().warning(f'Ошибка парсинга сообщения: {msg.data}. Ошибка: {e}')
    
    def publish_odometry(self, x, y, theta, vx, vth):
        """Публикация одометрии"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        
        # Позиция
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        # Ориентация (преобразуем угол в кватернион)
        q = self.angle_to_quaternion(theta)
        odom_msg.pose.pose.orientation = q
        
        # Скорость
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom_msg)
        
        # Публикация TF трансформации
        self.publish_tf_transform(x, y, theta)
    
    def publish_tf_transform(self, x, y, theta):
        """Публикация трансформации между odom и base_link"""
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
        """Публикация данных IMU (упрощенно - только угол и угловая скорость)"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Ориентация
        q = self.angle_to_quaternion(theta)
        imu_msg.orientation = q
        
        # Угловая скорость
        imu_msg.angular_velocity.z = vth
        
        self.imu_pub.publish(imu_msg)
    
    def publish_temperature(self, left_temp, right_temp):
        """Публикация температуры двигателей"""
        
        left_temp_msg = Temperature()
        left_temp_msg.header.stamp = self.get_clock().now().to_msg()
        left_temp_msg.temperature = left_temp
        self.left_temp_pub.publish(left_temp_msg)
        
        right_temp_msg = Temperature()
        right_temp_msg.header.stamp = self.get_clock().now().to_msg()
        right_temp_msg.temperature = right_temp
        self.right_temp_pub.publish(right_temp_msg)
    
    def publish_voltage(self, left_voltage, right_voltage):
        """Публикация напряжения двигателей"""
        left_voltage_msg = Float32()
        left_voltage_msg.data = left_voltage
        self.left_voltage_pub.publish(left_voltage_msg)
        
        right_voltage_msg = Float32()
        right_voltage_msg.data = right_voltage
        self.right_voltage_pub.publish(right_voltage_msg)

    def publish_load(self, left_load, right_load):
        """Публикация нагрузки двигателей"""
        left_load_msg = Float32()
        left_load_msg.data = left_load
        self.left_load_pub.publish(left_load_msg)
        
        right_load_msg = Float32()
        right_load_msg.data = right_load
        self.right_load_pub.publish(right_load_msg)

    def publish_position(self, left_position, right_position):
        """Публикация положений валов двигателей"""
        left_position_msg = Float32()
        left_position_msg.data = left_position
        self.left_position_pub.publish(left_position_msg)
        
        right_position_msg = Float32()
        right_position_msg.data = right_position
        self.right_position_pub.publish(right_position_msg)

    def angle_to_quaternion(self, angle):
        """Преобразование угла (в радианах) в кватернион"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(angle / 2.0)
        q.w = math.cos(angle / 2.0)
        return q

def main(args=None):
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