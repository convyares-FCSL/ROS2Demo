import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from collections import deque

# Headless-friendly: use Agg if no DISPLAY
import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt

class PressurePlotter(Node):
    def __init__(self):
        super().__init__('pressure_plotter')
        self.sub = self.create_subscription(Float64, '/tank_pressure', self.cb, 10)

        self.times = deque(maxlen=2000)
        self.values = deque(maxlen=2000)
        self.start = self.get_clock().now().nanoseconds / 1e9

        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], '-')
        self.ax.set_xlabel("Time [s]")
        self.ax.set_ylabel("Pressure [bar]")
        self.ax.grid(True)

        # If interactive backend, enable interactive draw; otherwise save PNG periodically
        self.interactive = plt.get_backend().lower() not in ('agg',)
        if self.interactive:
            plt.ion()
        self.png_path = '/tmp/tank_pressure_plot.png'
        self.save_timer = self.create_timer(1.0, self._save_png_if_headless)

    def cb(self, msg: Float64):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start
        self.times.append(t)
        self.values.append(msg.data)

        self.line.set_xdata(self.times)
        self.line.set_ydata(self.values)
        self.ax.relim(); self.ax.autoscale_view()

        if self.interactive:
            plt.draw(); plt.pause(0.01)

    def _save_png_if_headless(self):
        if not self.interactive:
            self.fig.savefig(self.png_path, dpi=120)
            # keep log light
            # self.get_logger().info(f"Saved {self.png_path}")

def main():
    rclpy.init()
    node = PressurePlotter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
