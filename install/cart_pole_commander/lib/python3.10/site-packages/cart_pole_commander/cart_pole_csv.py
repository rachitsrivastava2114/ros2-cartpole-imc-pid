"""
Cart-Pole CSV Error Logger
==========================
ROS 2 node that subscribes to /joint_states and logs the following to a CSV:

  - timestamp (seconds since node start)
  - pole_angle_error   = pole_angle_ref (0) − pole_angle   [rad]
  - cart_velocity_error = cart_vel_ref   (0) − cart_velocity [m/s]

The CSV is written to ~/cart_pole_error_<YYYYMMDD_HHMMSS>.csv on shutdown
(Ctrl-C or node destruction).

Usage (after colcon build + source):
    ros2 run cart_pole_commander cart_pole_csv
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CartPoleCsvLogger(Node):
    """Logs pole angle error and cart velocity error to a CSV file."""

    def __init__(self):
        super().__init__('cart_pole_csv_logger')

        # ── Joint index cache ─────────────────────────────────────────────
        self._cart_idx = None
        self._pole_idx = None

        # ── Reference values (desired = 0 for both) ──────────────────────
        self.pole_angle_ref = 0.0   # desired pole angle  [rad]
        self.cart_vel_ref   = 0.0   # desired cart velocity [m/s]

        # ── Data storage ─────────────────────────────────────────────────
        self._rows: list[dict] = []

        # ── Timing ───────────────────────────────────────────────────────
        self._start_time = self.get_clock().now()

        # ── Output path ──────────────────────────────────────────────────
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(
            os.path.expanduser('~'),
            f'cart_pole_error_{timestamp}.csv',
        )

        # ── Subscription ─────────────────────────────────────────────────
        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        self.get_logger().info(
            f'CSV error logger started. Data will be saved to: {self._csv_path}'
        )

    # ── Callback ──────────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        # Resolve joint indices once
        if self._cart_idx is None or self._pole_idx is None:
            try:
                self._cart_idx = list(msg.name).index('cart_joint')
                self._pole_idx = list(msg.name).index('pole_joint')
            except ValueError:
                return

        ci, pi = self._cart_idx, self._pole_idx

        try:
            # Elapsed time in seconds
            t = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

            # Pole angle (wrap to [-π, π])
            pole_angle = math.atan2(
                math.sin(msg.position[pi]),
                math.cos(msg.position[pi]),
            )

            # Cart velocity
            cart_vel = 0.0
            if len(msg.velocity) > max(ci, pi):
                cart_vel = msg.velocity[ci]

            # Errors (reference − actual)
            pole_angle_error = self.pole_angle_ref - pole_angle
            cart_vel_error   = self.cart_vel_ref   - cart_vel

            self._rows.append({
                'time_s':            round(t, 6),
                'pole_angle_error':  round(pole_angle_error, 6),
                'cart_velocity_error': round(cart_vel_error, 6),
            })

        except IndexError:
            pass

    # ── CSV writing ──────────────────────────────────────────────────────────

    def _write_csv(self):
        if not self._rows:
            self.get_logger().warn('No data collected — CSV not written.')
            return

        fieldnames = ['time_s', 'pole_angle_error', 'cart_velocity_error']

        with open(self._csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

        self.get_logger().info(
            f'CSV saved: {self._csv_path}  ({len(self._rows)} rows)'
        )

    # ── Clean shutdown ───────────────────────────────────────────────────────

    def destroy_node(self):
        self._write_csv()
        super().destroy_node()


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleCsvLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
