import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import asyncio

class IMUYawVisualizer(Node):
    def __init__(self):
        super().__init__('imu_yaw_visualizer')
        self.yaw_angles = []
        self.times = []
        self.x_positions = [0]
        self.y_positions = [0]
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10
        )
        self.figure, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.line1, = self.ax1.plot([], [], lw=2)
        self.line2, = self.ax2.plot([], [], lw=2)
        self.ax1.set_xlim(0, 300)
        self.ax1.set_ylim(-np.pi, np.pi)
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Yaw (radians)')

        self.ax2.set_xlim(-300, 300)
        self.ax2.set_ylim(-300, 300)
        self.ax2.set_xlabel('X position')
        self.ax2.set_ylabel('Y position')

        self.anim = FuncAnimation(
            self.figure,
            self.update_plot,
            init_func=self.init_plot,
            interval=1000,
            blit=True
        )

        plt.show(block=False)
        
    def listener_callback(self, msg):
        orientation = msg.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        euler_angles = self.quaternion_to_euler(qx, qy, qz, qw)
        yaw = euler_angles[2]  
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        self.times.append(elapsed_time)
        self.yaw_angles.append(yaw)

        velocity = 1.0  
        delta_time = 1.0  
        last_x, last_y = self.x_positions[-1], self.y_positions[-1]

        
        delta_x = velocity * np.cos(yaw) * delta_time
        delta_y = velocity * np.sin(yaw) * delta_time

        new_x = last_x + delta_x
        new_y = last_y + delta_y

        self.x_positions.append(new_x)
        self.y_positions.append(new_y)

        self.get_logger().info(f"Time: {elapsed_time:.2f}, Yaw: {yaw:.2f}, X: {new_x:.2f}, Y: {new_y:.2f}")

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def init_plot(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        return self.line1, self.line2

    def update_plot(self, frame):
        if not self.times or not self.yaw_angles:
            return self.line1, self.line2
        self.line1.set_data(self.times, self.yaw_angles)
        self.ax1.relim()
        self.ax1.autoscale_view()

        self.line2.set_data(self.x_positions, self.y_positions)
        self.ax2.relim()
        self.ax2.autoscale_view()

        return self.line1, self.line2

def main(args=None):
    rclpy.init(args=args)
    node = IMUYawVisualizer()

    async def run():
        while rclpy.ok():
            rclpy.spin_once(node)
            await asyncio.sleep(0.1)
            plt.pause(0.1)  

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
