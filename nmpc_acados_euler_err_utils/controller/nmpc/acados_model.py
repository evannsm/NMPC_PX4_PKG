"""
Quadrotor Model with Euler Angles for NMPC
==========================================

State: x = [p(3), v(3), euler(3)] = 9D
  - p: position [x, y, z] in world frame (z-up)
  - v: velocity [vx, vy, vz] in world frame
  - euler: [roll, pitch, yaw] ZYX convention

Control: u = [F, p, q, r] = 4D
  - F: collective thrust magnitude (N), along +body_z
  - [p, q, r]: body angular rates (rad/s)

This model uses stage-wise parameters for references to enable error-based cost.
Parameters: p = [p_ref(3), v_ref(3), euler_ref(3), u_ref(4)] = 13D
"""

from casadi import MX, vertcat, cos, sin, tan, atan2, mtimes
from acados_template import AcadosModel


class QuadrotorEulerModel:
    def __init__(self, mass: float):
        self.g = 9.806  # gravity magnitude (z-up -> gravity points -z)
        self.m = mass

    def dynamics(self):
        """
        Create the dynamics and error-based cost expressions.

        Returns:
            f_expl: explicit dynamics f(x, u)
            f_impl: implicit dynamics xdot - f(x, u) = 0
            x: state vector (9D)
            xdot: state derivative vector (9D)
            u: control vector (4D)
            p: parameter vector (13D) = [p_ref, v_ref, euler_ref, u_ref]
            cost_y_expr: error-based cost expression for stage cost
            cost_y_expr_e: error-based cost expression for terminal cost
        """
        # --- Helper functions ---
        def rotate_thrust(F, euler):
            """
            Map body thrust (along +b_z) to world frame (z-up).
            Uses Rz(yaw) * Ry(pitch) * Rx(roll), returns F * (R * e3).
            """
            roll  = euler[0]
            pitch = euler[1]
            yaw   = euler[2]

            sr, cr = sin(roll),  cos(roll)
            sp, cp = sin(pitch), cos(pitch)
            sy, cy = sin(yaw),   cos(yaw)

            # Third column of R_z(yaw) R_y(pitch) R_x(roll)
            Re3 = vertcat(
                cr*sp*cy + sr*sy,
                cr*sp*sy - sr*cy,
                cr*cp
            )
            return F * Re3

        def euler_rate_matrix(euler):
            """
            ZYX Euler-rate mapping:
            [roll_dot, pitch_dot, yaw_dot]^T = T(roll,pitch) @ [p, q, r]^T
            """
            roll  = euler[0]
            pitch = euler[1]

            sr, cr = sin(roll),  cos(roll)
            sp, cp = sin(pitch), cos(pitch)
            tanp = tan(pitch)

            T = MX(3, 3)
            T[0, 0] = 1.0
            T[0, 1] = sr * tanp
            T[0, 2] = cr * tanp

            T[1, 0] = 0.0
            T[1, 1] = cr
            T[1, 2] = -sr

            T[2, 0] = 0.0
            T[2, 1] = sr / cp
            T[2, 2] = cr / cp
            return T

        # --- States (9D) ---
        p     = MX.sym('p', 3)       # position [x,y,z]
        v     = MX.sym('v', 3)       # velocity
        euler = MX.sym('euler', 3)   # [roll, pitch, yaw]
        x     = vertcat(p, v, euler)

        # --- Inputs (4D) ---
        F = MX.sym('F')       # collective thrust magnitude (N)
        w = MX.sym('w', 3)    # body rates [p, q, r]
        u = vertcat(F, w)

        # --- State derivatives ---
        p_dot     = MX.sym('p_dot', 3)
        v_dot     = MX.sym('v_dot', 3)
        euler_dot = MX.sym('euler_dot', 3)
        xdot      = vertcat(p_dot, v_dot, euler_dot)

        # --- Parameters for references (13D) ---
        p_ref     = MX.sym('p_ref', 3)      # position reference
        v_ref     = MX.sym('v_ref', 3)      # velocity reference
        euler_ref = MX.sym('euler_ref', 3)  # euler reference [roll_ref, pitch_ref, yaw_ref]
        u_ref     = MX.sym('u_ref', 4)      # control reference [F_ref, p_ref, q_ref, r_ref]
        params    = vertcat(p_ref, v_ref, euler_ref, u_ref)

        # --- Dynamics (explicit) ---
        g_vec     = vertcat(0.0, 0.0, self.g)
        a_thrust  = rotate_thrust(F, euler) / self.m
        T_euler   = euler_rate_matrix(euler)

        f_expl = vertcat(
            v,                         # p_dot
            -a_thrust + g_vec,         # v_dot (thrust along -z body maps to world)
            mtimes(T_euler, w)         # euler_dot
        )

        # --- Implicit form for Acados ---
        f_impl = xdot - f_expl

        # --- Error-based cost expressions ---
        # Position error
        p_err = p - p_ref

        # Velocity error
        v_err = v - v_ref

        # Attitude error (roll and pitch are simple differences)
        roll_err  = euler[0] - euler_ref[0]
        pitch_err = euler[1] - euler_ref[1]

        # WRAPPED YAW ERROR using atan2(sin, cos) for proper angle wrapping
        yaw_diff = euler[2] - euler_ref[2]
        yaw_err = atan2(sin(yaw_diff), cos(yaw_diff))

        euler_err = vertcat(roll_err, pitch_err, yaw_err)

        # Control error
        u_err = u - u_ref

        # Stage cost output: [p_err(3), v_err(3), euler_err(3), u_err(4)] = 13D
        cost_y_expr = vertcat(p_err, v_err, euler_err, u_err)

        # Terminal cost output: [p_err(3), v_err(3), euler_err(3)] = 9D
        cost_y_expr_e = vertcat(p_err, v_err, euler_err)

        return (f_expl, f_impl, x, xdot, u, params, cost_y_expr, cost_y_expr_e)
