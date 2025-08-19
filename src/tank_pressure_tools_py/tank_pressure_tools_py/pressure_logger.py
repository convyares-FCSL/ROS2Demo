import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pathlib import Path
from time import strftime

class PressureLogger(Node):
    def __init__(self):
        super().__init__('pressure_logger')
        self.declare_parameter('csv_path', '/tmp/tank_pressure.csv')
        self.csv_path = Path(self.get_parameter('csv_path').get_parameter_value().string_value)
        self.get_logger().info(f'Logging to {self.csv_path}')
        # write header
        if not self.csv_path.exists():
            self.csv_path.write_text('stamp,pressure_bar\n')
        # sub
        self.sub = self.create_subscription(
            Float64, '/tank_pressure', self._cb, 10)

    def _cb(self, msg: Float64):
        now = self.get_clock().now().to_msg()
        stamp = f'{now.sec}.{now.nanosec:09d}'
        with self.csv_path.open('a') as f:
            f.write(f'{stamp},{msg.data:.6f}\n')

def main():
    rclpy.init()
    node = PressureLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
