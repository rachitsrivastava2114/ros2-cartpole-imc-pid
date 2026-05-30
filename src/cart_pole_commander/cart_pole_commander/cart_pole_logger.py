import math
import os
import csv
from datetime import datetime

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CartPoleLogger(Node):
    def __init__(self):
        super().__init__('cart_pole_logger')

        self.cart_index = None
        self.pole_index = None

        self.start_time = self.get_clock().now()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(
            os.path.expanduser("~"),
            f"cart_pole_plots_{timestamp}"
        )
        os.makedirs(self.save_dir, exist_ok=True)

        # Reference values from controller
        self.x_ref = 0.0
        self.theta_ref = 0.0

        # Data storage
        self.time_data = []

        self.cart_pos_data = []
        self.cart_vel_data = []
        self.pole_angle_data = []
        self.pole_vel_data = []

        self.cart_pos_ref_data = []
        self.pole_angle_ref_data = []

        self.cart_pos_error_data = []
        self.pole_angle_error_data = []

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.ref_sub = self.create_subscription(
            Float64MultiArray,
            '/cartpole_ref',
            self.ref_callback,
            10
        )

        self.get_logger().info(
            f"Logging started. Plots will be saved in: {self.save_dir}"
        )

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self.x_ref = msg.data[0]
            self.theta_ref = msg.data[1]

    def joint_state_callback(self, msg: JointState):
        if self.cart_index is None or self.pole_index is None:
            try:
                self.cart_index = msg.name.index('cart_joint')
                self.pole_index = msg.name.index('pole_joint')
            except ValueError:
                return

        try:
            t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

            cart_pos = msg.position[self.cart_index]

            pole_angle = math.atan2(
                math.sin(msg.position[self.pole_index]),
                math.cos(msg.position[self.pole_index])
            )

            if len(msg.velocity) > max(self.cart_index, self.pole_index):
                cart_vel = msg.velocity[self.cart_index]
                pole_vel = msg.velocity[self.pole_index]
            else:
                cart_vel = 0.0
                pole_vel = 0.0

            cart_pos_error = self.x_ref - cart_pos
            pole_angle_error = self.theta_ref - pole_angle

            self.time_data.append(t)

            self.cart_pos_data.append(cart_pos)
            self.cart_vel_data.append(cart_vel)
            self.pole_angle_data.append(pole_angle)
            self.pole_vel_data.append(pole_vel)

            self.cart_pos_ref_data.append(self.x_ref)
            self.pole_angle_ref_data.append(self.theta_ref)

            self.cart_pos_error_data.append(cart_pos_error)
            self.pole_angle_error_data.append(pole_angle_error)

        except IndexError:
            pass

    def save_csv(self):
        csv_path = os.path.join(self.save_dir, "cart_pole_log.csv")

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)

            writer.writerow([
                "time",
                "cart_position",
                "cart_velocity",
                "cart_position_reference",
                "cart_position_error",
                "pole_angle",
                "pole_angular_velocity",
                "pole_angle_reference",
                "pole_angle_error"
            ])

            for i in range(len(self.time_data)):
                writer.writerow([
                    self.time_data[i],
                    self.cart_pos_data[i],
                    self.cart_vel_data[i],
                    self.cart_pos_ref_data[i],
                    self.cart_pos_error_data[i],
                    self.pole_angle_data[i],
                    self.pole_vel_data[i],
                    self.pole_angle_ref_data[i],
                    self.pole_angle_error_data[i]
                ])

        self.get_logger().info(f"CSV saved: {csv_path}")

    def plot_and_save_data(self):
        self.get_logger().info("Saving plots...")

        # Plot 1: Actual states
        plt.figure(figsize=(12, 8))

        plt.subplot(2, 2, 1)
        plt.plot(self.time_data, self.cart_pos_data, label="Cart Position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title("Cart Position")
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 2)
        plt.plot(self.time_data, self.cart_vel_data, label="Cart Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.title("Cart Velocity")
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 3)
        plt.plot(self.time_data, self.pole_angle_data, label="Pole Angle")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (rad)")
        plt.title("Pole Angle")
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(self.time_data, self.pole_vel_data, label="Pole Angular Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")
        plt.title("Pole Angular Velocity")
        plt.grid()
        plt.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, "actual_states.png"), dpi=300)

        # Plot 2: Reference vs Actual
        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.plot(self.time_data, self.cart_pos_ref_data, "--", label="Reference")
        plt.plot(self.time_data, self.cart_pos_data, label="Actual")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title("Cart Position: Reference vs Actual")
        plt.grid()
        plt.legend()

        plt.subplot(1, 2, 2)
        plt.plot(self.time_data, self.pole_angle_ref_data, "--", label="Reference")
        plt.plot(self.time_data, self.pole_angle_data, label="Actual")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (rad)")
        plt.title("Pole Angle: Reference vs Actual")
        plt.grid()
        plt.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, "reference_vs_actual.png"), dpi=300)

        # Plot 3: Errors
        plt.figure(figsize=(12, 6))

        plt.subplot(1, 2, 1)
        plt.plot(self.time_data, self.cart_pos_error_data)
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m)")
        plt.title("Cart Position Error")
        plt.grid()

        plt.subplot(1, 2, 2)
        plt.plot(self.time_data, self.pole_angle_error_data)
        plt.xlabel("Time (s)")
        plt.ylabel("Error (rad)")
        plt.title("Pole Angle Error")
        plt.grid()

        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, "errors.png"), dpi=300)

        plt.show()

    def destroy_node(self):
        if len(self.time_data) > 0:
            self.save_csv()
            self.plot_and_save_data()
            self.get_logger().info(f"Plots saved in: {self.save_dir}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CartPoleLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
