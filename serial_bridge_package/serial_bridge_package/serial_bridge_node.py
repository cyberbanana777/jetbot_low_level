#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SerialBridgeNode(Node):
    """
    Узел ROS2 для моста между топиками ROS2 и последовательным портом.
    """

    def __init__(self):
        super().__init__('serial_bridge_node')

        # Объявление параметров для настройки порта
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ros_out_topic', 'serial/from')
        self.declare_parameter('ros_in_topic', 'serial/to')

        # Получение значений параметров
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        ros_out_topic = self.get_parameter('ros_out_topic').get_parameter_value().string_value
        ros_in_topic = self.get_parameter('ros_in_topic').get_parameter_value().string_value

        # Инициализация последовательного порта
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=0.5  # Таймаут для чтения (в секундах)
            )
            self.get_logger().info(f"Последовательный порт {serial_port} открыт на скорости {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Ошибка открытия порта {serial_port}: {e}")
            raise e

        # Создание Publisher для отправки данных ИЗ последовательного порта В ROS
        # Данные можно публиковать как String или как байты (UInt8MultiArray)
        self.pub_to_ros = self.create_publisher(String, ros_out_topic, 10)
        # self.pub_to_ros = self.create_publisher(UInt8MultiArray, ros_out_topic, 10) # Альтернатива для байт

        # Создание Subscriber для приема данных ИЗ ROS и отправки В последовательный порт
        self.sub_from_ros = self.create_subscription(
            String,
            ros_in_topic,
            self.ros_callback,
            10)
        # Аналогично, можно подписаться на UInt8MultiArray, если нужна отправка сырых байт

        # Флаг для управления рабочими потоками
        self.running = True

        # Запуск потока для чтения из последовательного порта
        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True  # Поток завершится при завершении главного
        self.read_thread.start()

        self.get_logger().info("Узел моста последовательного порта запущен. Ожидание данных...")

    def ros_callback(self, msg):
        """
        Callback для сообщений, полученных из топика ROS.
        Отправляет данные в последовательный порт.
        """
        try:
            # Если используется String
            data_to_send = msg.data   # Добавляем символ новой строки как маркер конца сообщения
            self.ser.write(data_to_send.encode('utf-8'))
            self.get_logger().debug(f"Отправлено в UART: {data_to_send.strip()}")

            # Если бы использовался UInt8MultiArray:
            # data_to_send = bytes(msg.data)
            # self.ser.write(data_to_send)
        except Exception as e:
            self.get_logger().error(f"Ошибка записи в последовательный порт: {e}")

    def read_from_serial(self):
        """
        Работа в отдельном потоке: чтение данных из последовательного порта
        и публикация их в топик ROS.
        """
        while self.running and rclpy.ok():
            try:
                # Чтение всех доступных данных или ожидание по таймауту
                if self.ser.in_waiting > 0:
                    # Чтение байтов
                    data_bytes = self.ser.read(self.ser.in_waiting)

                    # Вариант 1: Публикация как String (удобно для текста)
                    try:
                        data_str = data_bytes.decode('utf-8', errors='ignore').strip()
                        ros_msg = String()
                        ros_msg.data = data_str
                        self.pub_to_ros.publish(ros_msg)
                        self.get_logger().debug(f"Получено из UART (String): {data_str}")
                    except UnicodeDecodeError:
                        self.get_logger().warn("Не удалось декодировать данные в UTF-8.")

            except serial.SerialException as e:
                self.get_logger().error(f"Ошибка чтения из последовательного порта: {e}")
                break
            # time.sleep(0.001)  # Короткая пауза, чтобы не нагружать CPU

    def destroy_node(self):
        """
        Корректное завершение работы при остановке узла.
        """
        self.get_logger().info("Завершение работы узла моста...")
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Последовательный порт закрыт.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_bridge_node = SerialBridgeNode()

    try:
        rclpy.spin(serial_bridge_node)
    except KeyboardInterrupt:
        serial_bridge_node.get_logger().info("Узел остановлен пользователем (Ctrl+C).")
    finally:
        serial_bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()