# vehicle_driver_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# The erpSerial class from the original script is now part of this driver node
class erpSerial:
    def __init__(self, port):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
        )
        self.send_data = bytearray(14)

    def send_ctrl_cmd(self, speed, steer):
        self.send_data[0] = 0x53
        self.send_data[1] = 0x54
        self.send_data[2] = 0x58
        self.send_data[3] = 0x01
        self.send_data[4] = 0x00
        self.send_data[5] = 0x00
        self.send_data[6] = int(speed)
        self.send_data[7] = int(speed)
        steer_bytes = steer.to_bytes(2, byteorder='little', signed=True)
        self.send_data[8] = steer_bytes[0]
        self.send_data[9] = steer_bytes[1]
        self.send_data[10] = 0x00
        self.send_data[11] = 0x0D
        self.send_data[12] = 0x0A
        self.send_data[13] = self.calculate_checksum()
        self.ser.write(self.send_data)

    def calculate_checksum(self):
        checksum = 0
        for i in range(13):
            checksum += self.send_data[i]
        return checksum & 0xFF

class VehicleDriverNode(Node):
    def __init__(self):
        super().__init__('vehicle_driver_node')
        self.get_logger().info("Vehicle Driver Node Started")

        self.declare_parameter('serial.port', 'COM4')
        serial_port = self.get_parameter('serial.port').get_parameter_value().string_value

        try:
            self.erp_serial = erpSerial(serial_port)
            self.get_logger().info(f"Successfully connected to ERP42 on {serial_port}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to connect to serial port {serial_port}: {e}")
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        velocity = int(msg.linear.x)
        steering = int(msg.angular.z)  # Assuming this is the degree value

        self.get_logger().info(f"Sending to vehicle: Speed={velocity}, Steer={steering}")
        self.erp_serial.send_ctrl_cmd(velocity, steering)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
