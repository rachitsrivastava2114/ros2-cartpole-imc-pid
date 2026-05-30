"""
IMC-PID Cart-Pole Controller
============================
ROS 2 node — drop-in replacement for
  cart_pole_ws/src/cart_pole_commander/cart_pole_commander/cart_pole_controller.py

Theory: "Nonlinear and Adaptive Control Design — Project 1"
         Rachit Srivastava (23UEC600), LNMIIT Jaipur, March 2026

Physical parameters (from cart_pole.urdf.xacro)
------------------------------------------------
  M  = 1.0  kg   cart mass
  m  = 0.2  kg   pole mass
  l  = 0.5  m    pole HALF-length  (full length = 1.0 m in URDF)
  g  = 9.81 m/s²
  cart_joint  : prismatic, x-axis, limits ±2.0 m
  pole_joint  : revolute,  y-axis
  effort cmd  : /cart_effort_controller/commands  (effort on cart_joint)

Plant models  (Section 2.3 of report)
---------------------------------------
Both subsystems share the same 2nd-order UNSTABLE structure:

    G(s) = k / (τ²s² - 1)

  Angle subsystem  G_θ(s)  [force → pole angle]:
    τ_θ = sqrt( M·l / (g·(M+m)) )  =  0.206091 s
    k_θ = -1 / (g·(M+m))           = -0.084947

  Cart position subsystem  G_x(s)  [θ_ref → cart position]:
    τ_x = sqrt( l / g )             =  0.225762 s
    k_x = -1 / l                    = -2.000000

IMC filter  (Section 3.2 of report)
--------------------------------------
For the 2nd-order unstable plant a lead-lag IMC filter is used:

    f(s) = (αs + 1) / (λs + 1)²

with pole-zero cancellation condition:

    α = λ(λ + 2τ) / τ

Equivalent feedback PID gains:

    Kp = (α + τ) / [k(α - 2λ)]
    Ki =       1 / [k(α - 2λ)]
    Kd = τ·α   / [k(α - 2λ)]

A scalar control multiplier μ is folded into all gains at construction.

Architecture  (Section 4.2 of report — Cascaded)
--------------------------------------------------
  OUTER loop  (cart position → θ_ref):
    e_x    = x_ref - x
    θ_ref  = clip( Kp_x·e_x + Ki_x·∫e_x + Kd_x·ė_x ,  ±θ_ref_max )

  INNER loop  (θ_ref → force):
    e_θ    = θ_ref - θ
    u      = clip( Kp_θ·e_θ + Ki_θ·∫e_θ + Kd_θ·ė_θ ,  ±u_max )

Anti-windup  (Section 4.3):
    integral is clipped to  [u_min/2, u_max/2]  at every step.

Sign convention for this URDF
-------------------------------
  pole_joint axis = +Y
  θ > 0 means pole leans in +X direction.
  A positive cart force (+X) causes the pole to lean in the -X direction
  (reaction), so the feedback gain k_θ is negative — matching the
  sign used in the report's code listing (k = -1/(g*(M+m))).
  The cascade architecture handles the mixing automatically through the
  θ_ref signal: no separate sign-flip is needed.

Tuning (recommended starting point)
--------------------------------------
  λ_θ = 0.05   μ_θ = 0.5    → Kp≈-5.4, Ki≈-10.2, Kd≈-0.50
  λ_x = 0.50   μ_x = 0.08   → Kp≈-0.032, Ki≈-0.007, Kd≈-0.012
  θ_ref_max = 0.15 rad  (prevents unrealistic lean commands)

  To tune: adjust λ_θ first (inner loop), then λ_x (outer loop).
  Smaller λ → faster / more aggressive. Larger λ → slower / more robust.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ═════════════════════════════════════════════════════════════════════════════
# IMC-PID class  (exact implementation of the report's equations)
# ═════════════════════════════════════════════════════════════════════════════

class IMCPID:
    """
    IMC-PID for a 2nd-order unstable plant:  G(s) = k / (τ²s² - 1)

    Parameters
    ----------
    tau   : plant time constant  τ  [s]
    k     : plant static gain  k
    lam   : IMC filter time constant  λ  [s]  ← single tuning knob
    dt    : sample period  [s]
    output_min / output_max : saturation limits on final output
    control_multiplier      : scalar μ folded into all gains (Section 4.4)
    name  : label for logging
    """

    def __init__(
        self,
        tau: float,
        k: float,
        lam: float,
        dt: float,
        output_min: float = -20.0,
        output_max: float =  20.0,
        control_multiplier: float = 1.0,
        name: str = "pid",
    ):
        self.tau  = tau
        self.k    = k
        self.lam  = lam
        self.dt   = dt
        self.name = name
        self.output_min = output_min
        self.output_max = output_max

        # ── Compute gains analytically (Eq. 3.4–3.5 of report) ──────────
        alpha = lam * (lam + 2.0 * tau) / tau          # Eq. 3.5
        denom = k * (alpha - 2.0 * lam)                # common denominator

        self.kp = control_multiplier * (alpha + tau) / denom   # Eq. 3.4
        self.ki = control_multiplier *  1.0           / denom
        self.kd = control_multiplier * (tau * alpha)  / denom

        self.alpha = alpha

        # Anti-windup limits: clip integral to half the output range (Eq. 4.4)
        self.integral_min = output_min / 2.0
        self.integral_max = output_max / 2.0

        # Controller state
        self.integral   = 0.0
        self.prev_error = 0.0

    # ── Accessors ─────────────────────────────────────────────────────────────

    def get_gains(self):
        return self.kp, self.ki, self.kd

    def get_imc_params(self):
        return self.tau, self.k, self.alpha

    def reset(self):
        """Clear integrator and derivative memory."""
        self.integral   = 0.0
        self.prev_error = 0.0

    # ── One control step (Listing 5.1 of report) ──────────────────────────────

    def compute(self, error: float) -> float:
        """
        Execute one PID step.

        Parameters
        ----------
        error : setpoint − measurement

        Returns
        -------
        u : control output, clipped to [output_min, output_max]
        """
        # Integral with anti-windup clip  (Eq. 4.4 / Listing 5.1)
        raw_integral  = self.integral + error * self.dt
        self.integral = float(np.clip(raw_integral,
                                      self.integral_min,
                                      self.integral_max))

        # Derivative on error
        derivative    = (error - self.prev_error) / self.dt
        self.prev_error = error

        # PID sum
        raw_output = (self.kp * error
                      + self.ki * self.integral
                      + self.kd * derivative)

        return float(np.clip(raw_output, self.output_min, self.output_max))

    def describe(self) -> str:
        return (
            f"[{self.name}]  τ={self.tau:.4f}  k={self.k:.6f}  "
            f"λ={self.lam}  α={self.alpha:.6f}\n"
            f"           Kp={self.kp:.6f}  Ki={self.ki:.6f}  Kd={self.kd:.6f}"
        )


# ═════════════════════════════════════════════════════════════════════════════
# ROS 2 Node
# ═════════════════════════════════════════════════════════════════════════════

class CartPoleIMCPIDController(Node):
    """
    Cascaded IMC-PID controller matching the report (Architecture 2, Sec. 4.2).

    Subscribes : /joint_states
    Publishes  : /cart_effort_controller/commands

    Architecture
    ------------
      Outer loop: x_ref=0  →  [Cart IMC-PID]  →  θ_ref  (clipped)
      Inner loop: θ_ref    →  [Angle IMC-PID] →  force u
    """

    # ── Physical constants (URDF) ─────────────────────────────────────────────
    M = 1.0     # cart mass [kg]
    m = 0.2     # pole mass [kg]
    l = 0.5     # pole HALF-length [m]  (URDF full length = 1.0 m)
    g = 9.81    # m/s²

    def __init__(self):
        super().__init__('cart_pole_imc_pid_controller')

        # ── ROS interfaces ────────────────────────────────────────────────
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self._cmd_pub = self.create_publisher(
            Float64MultiArray, '/cart_effort_controller/commands', 10)

        self.dt = 0.01       # 100 Hz
        self._timer = self.create_timer(self.dt, self._loop)

        # ── Joint index cache ─────────────────────────────────────────────
        self._ci = self._pi = None
        self._ready = False

        # ── State estimates ───────────────────────────────────────────────
        self.x          = 0.0
        self.x_dot      = 0.0
        self.theta      = 0.0
        self.theta_dot  = 0.0

        # Velocity low-pass filter  (α = 0.35 → ~5.5 Hz cut-off at 100 Hz)
        self._vfa  = 0.35
        self._fxd  = 0.0
        self._ftd  = 0.0

        # ── Setpoints ─────────────────────────────────────────────────────
        self.x_ref     = 0.0     # desired cart position
        self.theta_ref = 0.0     # desired pole angle (set by outer loop)

        # ── Safety limits ─────────────────────────────────────────────────
        self.ANGLE_LIMIT    = 0.60    # rad  — safety stop threshold
        self.POSITION_LIMIT = 1.85    # m

        # ── θ_ref saturation  (Section 4.2.1 — prevents large lean cmds) ─
        self.THETA_REF_MAX = 0.15     # rad  (±0.15 rad for sufficient authority)

        # ── Derive plant model parameters ─────────────────────────────────
        # Angle subsystem  (Section 2.3.1)
        tau_theta = math.sqrt(self.M * self.l / (self.g * (self.M + self.m)))
        k_theta   = -1.0 / (self.g * (self.M + self.m))

        # Cart position subsystem  (Section 2.3.2)
        tau_x = math.sqrt(self.l / self.g)
        k_x   = -1.0 / self.l

        # ── IMC-PID Controllers ───────────────────────────────────────────
        # Tuning: λ_θ=0.05  μ_θ=0.5    (retuned for proper gain magnitude)
        #         λ_x=0.50  μ_x=0.08   (slow outer loop for cascade stability)
        #
        # Change only lam_theta and lam_x to retune.
        lam_theta = 0.1
        lam_x     = 1.0

        mu_theta  = 0.5      # control multiplier for angle loop
        mu_x      = 0.03     # control multiplier for cart loop

        # Velocity damping gain for outer loop (fights oscillation directly)
        self.kv_x = 0.05     # subtract kv_x * x_dot from theta_ref

        self.angle_pid = IMCPID(
            tau=tau_theta,
            k=k_theta,
            lam=lam_theta,
            dt=self.dt,
            output_min=-40.0,
            output_max= 40.0,
            control_multiplier=mu_theta,
            name="ANGLE",
        )

        self.cart_pid = IMCPID(
            tau=tau_x,
            k=k_x,
            lam=lam_x,
            dt=self.dt,
            output_min=-self.THETA_REF_MAX * 0.3,
            output_max= self.THETA_REF_MAX * 0.3,
            control_multiplier=mu_x,
            name="CART ",
        )

        self._n = 0

        # ── Start-up log ──────────────────────────────────────────────────
        self.get_logger().info("=" * 65)
        self.get_logger().info("  Cart-Pole IMC-PID  (2nd-order unstable plant, cascaded)")
        self.get_logger().info("=" * 65)
        self.get_logger().info(
            f"  Plant: M={self.M} kg  m={self.m} kg  l={self.l} m  g={self.g}"
        )
        self.get_logger().info(self.angle_pid.describe())
        self.get_logger().info(self.cart_pid.describe())
        self.get_logger().info(
            f"  θ_ref clipped to ±{self.THETA_REF_MAX} rad"
        )

    # ── Joint state callback ──────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        # Resolve joint indices once
        if self._ci is None:
            try:
                self._ci = list(msg.name).index('cart_joint')
                self._pi = list(msg.name).index('pole_joint')
            except ValueError:
                return

        ci, pi = self._ci, self._pi
        try:
            self.x     = msg.position[ci]
            self.theta = math.atan2(
                math.sin(msg.position[pi]),
                math.cos(msg.position[pi])
            )

            raw_xd = raw_td = 0.0
            if len(msg.velocity) > max(ci, pi):
                raw_xd = msg.velocity[ci]
                raw_td = msg.velocity[pi]

            # Exponential low-pass filter on velocities
            a          = self._vfa
            self._fxd  = a * raw_xd + (1.0 - a) * self._fxd
            self._ftd  = a * raw_td + (1.0 - a) * self._ftd

            self.x_dot     = self._fxd
            self.theta_dot = self._ftd
            self._ready    = True

        except IndexError:
            self._ready = False

    # ── Main control loop ─────────────────────────────────────────────────────

    def _loop(self):
        if not self._ready:
            return

        # ── Safety stop ───────────────────────────────────────────────────
        if abs(self.theta) > self.ANGLE_LIMIT or abs(self.x) > self.POSITION_LIMIT:
            self.angle_pid.reset()
            self.cart_pid.reset()
            self._pub(0.0)
            self._n += 1
            if self._n % 50 == 0:
                self.get_logger().warn(
                    f"SAFETY STOP  |θ|={abs(self.theta):.3f} rad  "
                    f"|x|={abs(self.x):.3f} m"
                )
            return

        # ── OUTER LOOP: cart position → θ_ref  (Eq. 4.2) ─────────────────
        e_x = self.x_ref - self.x
        raw_theta_ref = -self.cart_pid.compute(e_x)

        # Velocity damping: opposes cart motion to kill oscillation growth
        vel_damp = -self.kv_x * self.x_dot

        theta_ref = float(np.clip(
            raw_theta_ref + vel_damp,
            -self.THETA_REF_MAX, self.THETA_REF_MAX
        ))

        # ── INNER LOOP: θ_ref → force  (Eq. 4.3) ─────────────────────────
        e_theta = theta_ref - self.theta
        u = self.angle_pid.compute(e_theta)

        self._pub(u)

        # ── Periodic log ──────────────────────────────────────────────────
        self._n += 1
        if self._n % 50 == 0:
            self.get_logger().info(
                f"x={self.x:+.3f}m  ẋ={self.x_dot:+.3f}  |  "
                f"θ={self.theta:+.4f}rad  θ̇={self.theta_dot:+.4f}  |  "
                f"e_x={e_x:+.3f}  θ_ref={theta_ref:+.4f}  "
                f"e_θ={e_theta:+.4f}  u={u:+.4f}N"
            )

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _pub(self, effort: float):
        msg = Float64MultiArray()
        msg.data = [float(effort)]
        self._cmd_pub.publish(msg)


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleIMCPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
