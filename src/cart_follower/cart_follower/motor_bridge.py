import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track_width', 0.50)      # m, placeholder
        self.declare_parameter('max_wheel_speed', 0.60)  # m/s, placeholder
        self.declare_parameter('wheel_radius', 0.18)     # m, placeholder
        self.declare_parameter('cmd_timeout', 0.5)       # s

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.B = self.get_parameter('track_width').value
        self.vmax = self.get_parameter('max_wheel_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # opening the port resets the Uno; wait for boot
        self.get_logger().info(f'Opened {port} @ {baud}')

        self.v = 0.0
        self.w = 0.0
        self.last_cmd = 0.0
        self.running = True

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/wheel_rpm', 10)
        self.create_timer(0.05, self.send_loop)  # 20 Hz

        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = time.monotonic()

    def send_loop(self):
        if time.monotonic() - self.last_cmd > self.cmd_timeout:
            line = 'S\n'
        else:
            v_l = self.v - self.w * self.B / 2.0
            v_r = self.v + self.w * self.B / 2.0
            left = self.scale(v_l)
            right = self.scale(v_r)
            line = f'M,{left},{right}\n'
        try:
            self.ser.write(line.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def scale(self, v):
        x = int(round(v / self.vmax * 1000.0))
        return max(-1000, min(1000, x))

    def read_loop(self):
        while self.running:
            try:
                raw = self.ser.readline().decode(errors='ignore').strip()
            except serial.SerialException:
                break
            if not raw.startswith('F,'):
                continue
            parts = raw.split(',')
            try:
                rpm = [float(p) for p in parts[1:5]]
            except ValueError:
                continue
            msg = Float32MultiArray()
            msg.data = rpm
            self.rpm_pub.publish(msg)

    def shutdown(self):
        self.running = False
        try:
            self.ser.write(b'S\n')
            self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()