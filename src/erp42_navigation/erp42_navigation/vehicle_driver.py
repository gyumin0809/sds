import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# erpSerial 클래스는 변경할 필요가 없습니다.
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
        steer_bytes = int(steer).to_bytes(2, byteorder='little', signed=True)
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

        self.declare_parameter('serial.port', '/dev/ttyUSB0')
        serial_port = self.get_parameter('serial.port').get_parameter_value().string_value

        try:
            self.erp_serial = erpSerial(serial_port)
            self.get_logger().info(f"Successfully connected to ERP42 on {serial_port}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to connect to serial port {serial_port}: {e}")
            # **수정 1: rclpy.shutdown() 대신 예외를 발생시켜서 생성 실패를 알립니다.**
            raise e

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        velocity = int(msg.linear.x)
        steering = int(msg.angular.z)

        self.get_logger().info(f"Sending to vehicle: Speed={velocity}, Steer={steering}")
        self.erp_serial.send_ctrl_cmd(velocity, steering)

    # **수정 3: 노드 종료 시 시리얼 포트를 닫는 함수 추가**
    def destroy_node(self):
        self.get_logger().info("Closing serial port.")
        if hasattr(self, 'erp_serial') and self.erp_serial.ser.is_open:
            self.erp_serial.ser.close()
        super().destroy_node()

# **수정 2: main 함수 구조 변경**
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        # 노드 생성을 try 블록 안에서 수행
        node = VehicleDriverNode()
        rclpy.spin(node)
    except Exception as e:
        # 노드 생성 중 발생한 예외(시리얼 포트 연결 실패 등)를 여기서 처리
        print(f"An error occurred during node initialization or spin: {e}")
    finally:
        # 노드가 성공적으로 생성되었다면 종료 시 destroy_node 호출
        if node is not None:
            node.destroy_node()
        # rclpy가 실행 중이면 안전하게 종료
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
