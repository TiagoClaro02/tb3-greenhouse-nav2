#!/usr/bin/env python3

import sys
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is required: pip install matplotlib --break-system-packages", file=sys.stderr)
    sys.exit(1)


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.xs = []
        self.ys = []
        self.map_msg = None

        # /odom publishes much faster than AMCL corrects (which only updates
        # every update_min_d/update_min_a of motion) -- velocity is recorded
        # here, not from /amcl_pose, so oscillation that's invisible in the
        # coarser trajectory plot still shows up clearly over time.
        self.vel_t = []
        self.linear_x = []
        self.angular_z = []
        self._t0 = None

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10
        )
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)

        # map_server publishes /map as transient_local -- match that QoS or
        # a late-joining subscriber never receives the latched message.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        self.get_logger().info('Recording /amcl_pose and /odom. Ctrl+C to save and exit.')

    def _on_pose(self, msg):
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def _on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        self.vel_t.append(t - self._t0)
        self.linear_x.append(msg.twist.twist.linear.x)
        self.angular_z.append(msg.twist.twist.angular.z)

    def _on_map(self, msg):
        if self.map_msg is None:
            self.get_logger().info(
                f'Got map: {msg.info.width}x{msg.info.height} @ {msg.info.resolution} m/cell'
            )
        self.map_msg = msg

    def save_plot(self):
        if not self.xs:
            self.get_logger().warn('No pose samples recorded -- nothing to plot.')
            return
        if self.map_msg is None:
            self.get_logger().warn('No /map received -- plotting trajectory without map background.')

        fig = plt.figure(figsize=(10, 10))
        gs = fig.add_gridspec(2, 2, height_ratios=[2, 1])
        ax_traj = fig.add_subplot(gs[0, :])
        ax_lin = fig.add_subplot(gs[1, 0])
        ax_ang = fig.add_subplot(gs[1, 1])

        if self.map_msg is not None:
            info = self.map_msg.info
            grid = np.array(self.map_msg.data, dtype=np.int8).reshape(info.height, info.width)
            extent = [
                info.origin.position.x,
                info.origin.position.x + info.width * info.resolution,
                info.origin.position.y,
                info.origin.position.y + info.height * info.resolution,
            ]
            ax_traj.imshow(grid, cmap='gray_r', origin='lower', extent=extent, vmin=-1, vmax=100)

        ax_traj.plot(self.xs, self.ys, '-', color='red', linewidth=1.2, label='trajectory')
        ax_traj.plot(self.xs[0], self.ys[0], 'go', markersize=8, label='start')
        ax_traj.plot(self.xs[-1], self.ys[-1], 'rs', markersize=8, label='end')
        ax_traj.set_xlabel('x (m)')
        ax_traj.set_ylabel('y (m)')
        ax_traj.set_title(f'Trajectory ({len(self.xs)} pose samples)')
        ax_traj.legend()
        ax_traj.set_aspect('equal')

        if self.vel_t:
            ax_lin.plot(self.vel_t, self.linear_x, color='blue', linewidth=0.8)
            ax_lin.axhline(0, color='gray', linewidth=0.5)
            ax_lin.set_xlabel('time (s)')
            ax_lin.set_ylabel('linear.x (m/s)')
            ax_lin.set_title('Linear velocity')

            ax_ang.plot(self.vel_t, self.angular_z, color='darkorange', linewidth=0.8)
            ax_ang.axhline(0, color='gray', linewidth=0.5)
            ax_ang.set_xlabel('time (s)')
            ax_ang.set_ylabel('angular.z (rad/s)')
            ax_ang.set_title('Angular velocity (oscillation shows up here directly)')
        else:
            ax_lin.text(0.5, 0.5, 'no /odom samples', ha='center', va='center')
            ax_ang.text(0.5, 0.5, 'no /odom samples', ha='center', va='center')

        fig.tight_layout()
        filename = f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        fig.savefig(filename, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Saved {filename}')


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
