import math
import os
from datetime import datetime

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CartPoleLogger(Node):
    def __init__(self):
        super().__init__('cart_pole_logger')

        self.cart_index = None
        self.pole_index = None

        self.start_time = self.get_clock().now()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(os.path.expanduser("~"), f"cart_pole_plots_{timestamp}")
        os.makedirs(self.save_dir, exist_ok=True)

        # Time
        self.time_data = []

        # Actual states
        self.cart_pos_data = []
        self.cart_vel_data = []
        self.pole_angle_data = []
        self.pole_vel_data = []

        # Reference states
        self.cart_pos_ref_data = []
        self.cart_vel_ref_data = []
        self.pole_angle_ref_data = []
        self.pole_vel_ref_data = []

        # Error states
        self.cart_pos_error_data = []
        self.cart_vel_error_data = []
        self.pole_angle_error_data = []
        self.pole_vel_error_data = []

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info(f'Logging started. Plots will be saved in: {self.save_dir}')

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

            # References
            cart_pos_ref = 0.0
            cart_vel_ref = 0.0
            pole_angle_ref = 0.0
            pole_vel_ref = 0.0

            # Errors
            cart_pos_error = cart_pos_ref - cart_pos
            cart_vel_error = cart_vel_ref - cart_vel
            pole_angle_error = pole_angle_ref - pole_angle
            pole_vel_error = pole_vel_ref - pole_vel

            # Store time
            self.time_data.append(t)

            # Store actual
            self.cart_pos_data.append(cart_pos)
            self.cart_vel_data.append(cart_vel)
            self.pole_angle_data.append(pole_angle)
            self.pole_vel_data.append(pole_vel)

            # Store reference
            self.cart_pos_ref_data.append(cart_pos_ref)
            self.cart_vel_ref_data.append(cart_vel_ref)
            self.pole_angle_ref_data.append(pole_angle_ref)
            self.pole_vel_ref_data.append(pole_vel_ref)

            # Store error
            self.cart_pos_error_data.append(cart_pos_error)
            self.cart_vel_error_data.append(cart_vel_error)
            self.pole_angle_error_data.append(pole_angle_error)
            self.pole_vel_error_data.append(pole_vel_error)

        except IndexError:
            pass

    def plot_and_save_data(self):
        self.get_logger().info("Saving plots...")

        # Figure 1: Actual states
        plt.figure(figsize=(12, 8))

        
        plt.subplot(2, 2, 1)
        plt.plot(self.time_data, self.cart_vel_data, label='Actual')
        plt.title("Cart Position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.grid()
        plt.legend()

        
        plt.subplot(2, 2, 2)
        plt.plot(self.time_data, self.pole_vel_data, label='Actual')
        plt.title("Pole Angle")
        plt.xlabel("Time (s)")
        plt.ylabel("Pole Angle (rad)")
        plt.grid()
        plt.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, "actual_states.png"), dpi=300)

        # Figure 2: Reference vs Actual
        plt.figure(figsize=(12, 8))

        plt.subplot(2, 2, 1)
        plt.plot(self.time_data, self.cart_pos_ref_data, '--', label='Reference')
        plt.plot(self.time_data, self.cart_pos_data, label='Actual')
        plt.title("Cart Position: Reference vs Actual")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 2)
        plt.plot(self.time_data, self.cart_vel_ref_data, '--', label='Reference')
        plt.plot(self.time_data, self.cart_vel_data, label='Actual')
        plt.title("Cart Velocity: Reference vs Actual")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 3)
        plt.plot(self.time_data, self.pole_angle_ref_data, '--', label='Reference')
        plt.plot(self.time_data, self.pole_angle_data, label='Actual')
        plt.title("Pole Angle: Reference vs Actual")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (rad)")
        plt.grid()
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(self.time_data, self.pole_vel_ref_data, '--', label='Reference')
        plt.plot(self.time_data, self.pole_vel_data, label='Actual')
        plt.title("Pole Angular Velocity: Reference vs Actual")
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")
        plt.grid()
        plt.legend()

        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, "reference_vs_actual.png"), dpi=300)

        # Figure 3: Errors
        plt.figure(figsize=(12, 8))

        plt.subplot(2, 2, 1)
        plt.plot(self.time_data, self.cart_pos_error_data)
        plt.title("Cart Position Error")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m)")
        plt.grid()

        plt.subplot(2, 2, 2)
        plt.plot(self.time_data, self.cart_vel_error_data)
        plt.title("Cart Velocity Error")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m/s)")
        plt.grid()

        plt.subplot(2, 2, 3)
        plt.plot(self.time_data, self.pole_angle_error_data)
        plt.title("Pole Angle Error")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (rad)")
        plt.grid()

        plt.subplot(2, 2, 4)
        plt.plot(self.time_data, self.pole_vel_error_data)
        plt.title("Pole Angular Velocity Error")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (rad/s)")
        plt.grid()

        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, "errors.png"), dpi=300)

        plt.show()

    def destroy_node(self):
        if len(self.time_data) > 0:
            self.plot_and_save_data()
            self.get_logger().info(f'Plots saved in: {self.save_dir}')
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


if __name__ == '__main__':
    main()

