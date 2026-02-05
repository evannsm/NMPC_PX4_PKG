"""
ROS2 Node for NMPC with Euler State and Wrapped Yaw Error
=========================================================

This node implements NMPC control for a quadrotor using:
- Euler angle state representation [p(3), v(3), euler(3)]
- Error-based cost with wrapped yaw error (atan2 formulation)
- Stage-wise parameters for reference passing
"""

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleRatesSetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleOdometry,
    RcChannels
)
from nmpc_acados_euler_err_utils.px4_utils.core_funcs import (
    engage_offboard_mode,
    arm,
    land,
    disarm,
    publish_offboard_control_heartbeat_signal_position,
    publish_offboard_control_heartbeat_signal_bodyrate
)
from quad_platforms import (
    PlatformType,
    PlatformConfig,
    PLATFORM_REGISTRY
)
from quad_trajectories import (
    TrajContext,
    TrajectoryType,
    TRAJ_REGISTRY,
    generate_reference_trajectory
)
from nmpc_acados_euler_err_utils.controller.nmpc import (
    QuadrotorEulerModel,
    QuadrotorEulerErrMPC
)

from nmpc_acados_euler_err_utils.main_utils import BANNER
from nmpc_acados_euler_err_utils.transformations.adjust_yaw import adjust_yaw
from nmpc_acados_euler_err_utils.px4_utils.flight_phases import FlightPhase

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from pyJoules.handler.csv_handler import CSVHandler
from pyJoules.device.rapl_device import RaplPackageDomain
from pyJoules.energy_meter import EnergyContext

from Logger import LogType, VectorLogType  # pyright: ignore[reportMissingImports, reportAttributeAccessIssue]

GRAVITY: float = 9.806


class OffboardControl(Node):
    def __init__(self, platform_type: PlatformType, trajectory: TrajectoryType = TrajectoryType.HOVER,
                 hover_mode: int | None = None, double_speed: bool = True, short: bool = False,
                 spin: bool = False, pyjoules: bool = False, csv_handler: CSVHandler | None = None) -> None:

        super().__init__('nmpc_euler_err_offboard_node')
        self.get_logger().info(f"{BANNER}Initializing ROS 2 node: '{self.__class__.__name__}'{BANNER}")
        self.sim = platform_type == PlatformType.SIM
        self.platform_type = platform_type
        self.trajectory_type = trajectory
        self.hover_mode = hover_mode
        self.double_speed = double_speed
        self.short = short
        self.spin = spin
        self.pyjoules_on = pyjoules
        if self.pyjoules_on:
            print("PyJoules energy monitoring ENABLED")
            self.csv_handler = csv_handler

        # Initialize platform configuration using dependency injection
        platform_class = PLATFORM_REGISTRY[self.platform_type]
        self.platform: PlatformConfig = platform_class()

        # Map trajectory string to TrajectoryType enum
        trajectory_map = {traj_type.value: traj_type for traj_type in TrajectoryType}
        if trajectory not in trajectory_map:
            raise ValueError(f"Unknown trajectory: {trajectory}. Available: {list(trajectory_map.keys())}")
        self.ref_type = trajectory_map[trajectory]
        print(f"\n[Trajectory] Main trajectory type: {self.ref_type.name}")

        # --- Set up Logging Arrays ---
        if True:
            print("Data logging is ON")
            self.data_log_timer_period = .1
            self.first_log = True
            if self.first_log:
                self.first_log = False
                self.get_logger().info("Starting data logging.")
                self.platform_logtype = LogType("platform", 0)
                self.trajectory_logtype = LogType("trajectory", 2)
                self.traj_double_logtype = LogType("traj_double", 3)
                self.traj_short_logtype = LogType("traj_short", 4)
                self.traj_spin_logtype = LogType("traj_spin", 5)

                self.platform_logtype.append(self.platform_type.value.upper())
                self.trajectory_logtype.append(self.ref_type.name)
                self.traj_double_logtype.append("DblSpd" if self.double_speed else "NormSpd")
                self.traj_short_logtype.append("Short" if self.short else "Not Short")
                self.traj_spin_logtype.append("Spin" if self.spin else "NoSpin")

            # Time logs
            self.program_time_logtype = LogType("time", 6)
            self.trajectory_time_logtype = LogType("traj_time", 7)
            self.reference_time_logtype = LogType("ref_time", 8)
            self.comptime_logtype = LogType("comptime", 9)

            # State logs
            self.x_logtype = LogType("x", 10)
            self.y_logtype = LogType("y", 11)
            self.z_logtype = LogType("z", 12)
            self.yaw_logtype = LogType("yaw", 13)

            # Reference logs
            self.xref_logtype = LogType("x_ref", 20)
            self.yref_logtype = LogType("y_ref", 21)
            self.zref_logtype = LogType("z_ref", 22)
            self.yawref_logtype = LogType("yaw_ref", 22)

            # Control input logs (normalized)
            self.throttle_input_logtype = LogType("throttle_input", 26)
            self.p_input_logtype = LogType("p_input", 27)
            self.q_input_logtype = LogType("q_input", 28)
            self.r_input_logtype = LogType("r_input", 29)

            self.cbf_logtype = VectorLogType("cbf_term", 30, ['thrust_cbf', 'roll_cbf', 'pitch_cbf', 'yaw_cbf'])

        # ----------------------- ROS2 Node Stuff --------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------- Publishers --------------------------
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.rates_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # ----------------------- Subscribers --------------------------
        self.mocap_k: int = -1
        self.full_rotations: int = 0
        self.mocap_initialized: bool = False
        self.prev_mocap_psi: float = 0.0
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback, qos_profile)

        self.in_offboard_mode: bool = False
        self.armed: bool = False
        self.in_land_mode: bool = False
        self.vehicle_status = None
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        self.offboard_mode_rc_switch_on = True if self.sim else False
        self.mode_channel = 5
        self.rc_channels_subscriber = self.create_subscription(
            RcChannels, '/fmu/out/rc_channels',
            self.rc_channel_callback, qos_profile)

        # ----------------------- Timers --------------------------
        self.data_log_timer_period = 1.0 / 10.0
        self.data_log_timer = self.create_timer(self.data_log_timer_period,
                                                self.data_log_timer_callback) if not self.pyjoules_on else None

        self.offboard_setpoint_counter = 0
        self.offboard_timer_period = 1.0 / 10.0
        self.timer = self.create_timer(self.offboard_timer_period,
                                       self.offboard_mode_timer_callback)

        self.publish_control_timer_period = 1.0 / 100.0
        self.publish_control_timer = self.create_timer(self.publish_control_timer_period,
                                                       self.publish_control_timer_callback)

        self.compute_control_timer_period = 1.0 / 100.0
        self.compute_control_timer = self.create_timer(self.compute_control_timer_period,
                                                       self.compute_control_timer_callback)

        # ----------------------- Set up Flight Phases and Time --------------------------
        self.T0 = time.time()
        self.program_time: float = 0.0
        self.cushion_period = 10.0
        self.flight_period = 30.0
        self.land_time = self.flight_period + 2 * self.cushion_period
        self.flight_phase = self.get_phase()
        print(f"Flight time: {self.land_time}s")

        # ----------------------- Initialize Control --------------------------
        self.HOVER_HEIGHT = 3.0 if self.sim else 0.7
        self.LAND_HEIGHT = 0.6 if self.sim else 0.45

        self.trajectory_started: bool = False
        self.trajectory_time: float = 0.0
        self.reference_time: float = 0.0
        self.T_LOOKAHEAD: float = 1.2

        self.first_thrust = self.platform.mass * GRAVITY
        self.last_input = np.array([self.first_thrust, 0.0, 0.0, 0.0])
        self.normalized_input = [self.platform.get_throttle_from_force(self.first_thrust), 0.0, 0.0, 0.0]

        # ===== MPC Setup (Error-Based Euler) =====
        print("\n[Quadrotor MPC] Initializing NMPC with wrapped yaw error...")
        model = QuadrotorEulerModel(mass=self.platform.mass)
        _, _, x_, _, u_, _, _, _ = model.dynamics()
        self.nx = x_.size()[0]
        self.nu = u_.size()[0]

        generate_c_code = False
        horizon = 2.0
        num_steps = 50
        self.horizon = horizon
        self.num_steps = num_steps
        self.mpc_solver = QuadrotorEulerErrMPC(
            generate_c_code=generate_c_code,
            quadrotor=model,
            horizon=horizon,
            num_steps=num_steps
        )

        # Buffer for predicted control sequence
        self.control_buffer = np.zeros((num_steps, self.nu))
        self.buffer_index = 0
        self.buffer_valid = False

        # Initial test solve
        pos0 = np.array([0.01, 0.02, -self.HOVER_HEIGHT])
        vel0 = np.array([0.01, -0.03, 0.05])
        euler0 = np.array([0.01, 0.02, 0.03])
        self._state0 = np.hstack((pos0, vel0, euler0))

        pos_des = np.array([3.0, 3.0, -5.0])
        vel_des = np.array([-0.01, 0.03, 0.05])
        euler_des = np.array([0.01, 0.01, -0.01])
        self._ref0 = np.hstack((pos_des, vel_des, euler_des))
        self.reff = np.tile(self._ref0, (num_steps, 1))

        print(f"[Quadrotor MPC] Initial reference trajectory:\n{self._ref0}")
        t0 = time.time()
        u_nmpc, x_mpc, status = self.mpc_solver.solve_mpc_control(
            self._state0, self.reff, self.last_input, nx=self.nx, nu=self.nu, verbose=False)
        t1 = time.time()
        print(f"[Quadrotor MPC] Initial MPC solve status: {status}, input: {u_nmpc[0, :]}\n")
        print(f"[Quadrotor MPC] Initial MPC solve time: {t1 - t0:.4f}s, Good for {1.0 / (t1 - t0):.2f} Hz control loop\n")

        # ----------------------- Initialize Trajectory --------------------------
        self.jit_compile_trajectories()
        print("[Offboard Control Node] Node initialized successfully!\n")
        time.sleep(3)

        self.T0 = time.time()

    def time_and_compare(self, func, *args, **kwargs):
        """Function to time a function call and return its output along with the time taken."""
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        elapsed_time = end_time - start_time
        return result, elapsed_time

    def jit_compile_trajectories(self) -> None:
        """Pre-compile JIT trajectory functions."""
        print("\n[JIT Compilation] Pre-compiling trajectory functions...")

        print("  Compiling hover trajectory...")
        ref, hover_total_time_1 = self.time_and_compare(
            self.generate_ref_trajectory,
            TrajectoryType.HOVER,
            hover_mode=self.hover_mode
        )

        print("  Compiling regular trajectory...")
        ref, regular_total_time_1 = self.time_and_compare(
            self.generate_ref_trajectory,
            self.ref_type,
        )

        print(f"  Testing JIT-compiled trajectory functions...")
        ref, hover_total_time_2 = self.time_and_compare(
            self.generate_ref_trajectory,
            TrajectoryType.HOVER,
            hover_mode=self.hover_mode
        )

        print(f"  Hover trajectory (JIT):\n{ref[0][0,:]}")
        print(f"  Hover speed up: {(hover_total_time_1) / (hover_total_time_2):.2f}x")

        ref, regular_total_time_2 = self.time_and_compare(
            self.generate_ref_trajectory,
            self.ref_type,
        )
        print(f"  Regular trajectory (JIT):\n{ref[0][0,:]}")
        print(f"  Regular speed up: {(regular_total_time_1) / (regular_total_time_2):.2f}x")

        r, rdot = ref
        print(f"{r[0,:]}")
        print(f"{rdot[0,:]}")
        # turn the whole history of r and rdot into [x,y,z,vx,vy,vz,0.,0.,yaw] histories using numpy stacking given that r = [x,y,z,yaw] and rdot = [vx,vy,vz,yaw_rate]
        ref_whole = np.hstack((r[:, 0:3], rdot[:, 0:3], np.zeros((rdot.shape[0], 2)), r[:, -1:]))
        print(f"{ref_whole[0,:]=}")
        print(f"{ref_whole.shape=}")
        # exit(0)
    # ========== Subscriber Callbacks ==========
    def vehicle_odometry_callback(self, msg):
        """Process odometry and convert to Euler state."""
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = msg.position[2]

        self.vx = msg.velocity[0]
        self.vy = msg.velocity[1]
        self.vz = msg.velocity[2]

        self.wx = msg.angular_velocity[0]
        self.wy = msg.angular_velocity[1]
        self.wz = msg.angular_velocity[2]

        self.qw = msg.q[0]
        self.qx = msg.q[1]
        self.qy = msg.q[2]
        self.qz = msg.q[3]

        orientation = R.from_quat(msg.q, scalar_first=True)
        self.roll, self.pitch, self._yaw = orientation.as_euler('xyz', degrees=False)
        self.yaw = adjust_yaw(self, self._yaw)

        self.position = np.array([self.x, self.y, self.z])
        self.velocity = np.array([self.vx, self.vy, self.vz])
        self.euler_angle_raw = np.array([self.roll, self.pitch, self._yaw])
        self.euler_angle_total_yaw = np.array([self.roll, self.pitch, self.yaw])
        self.quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])
        self.angular_velocities = np.array([self.wx, self.wy, self.wz])

        self.state_output = np.hstack((self.position, self.yaw))
        self.nmpc_state = np.hstack((self.position, self.velocity, self.euler_angle_total_yaw))

        self.get_logger().info(f'\nState output: {self.state_output}', throttle_duration_sec=0.3)

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        self.in_offboard_mode = (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.armed = (self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.in_land_mode = (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)

    def rc_channel_callback(self, rc_channels):
        flight_mode = rc_channels.channels[self.mode_channel - 1]
        self.offboard_mode_rc_switch_on = True if flight_mode >= 0.75 else False

    # ========== Timer Callbacks ==========
    def get_phase(self) -> FlightPhase:
        """Determine the current flight phase based on elapsed time."""
        if self.program_time < self.cushion_period:
            return FlightPhase.HOVER
        elif self.program_time < self.cushion_period + self.flight_period:
            return FlightPhase.CUSTOM
        elif self.program_time < self.land_time:
            return FlightPhase.RETURN
        else:
            return FlightPhase.LAND

    def time_before_next_phase(self, current_phase: FlightPhase) -> float:
        """Get the time remaining before the next flight phase."""
        if current_phase == FlightPhase.HOVER:
            return self.cushion_period - self.program_time
        elif current_phase == FlightPhase.CUSTOM:
            return (self.cushion_period + self.flight_period) - self.program_time
        elif current_phase == FlightPhase.RETURN:
            return self.land_time - self.program_time
        else:
            return 0.0

    def killswitch_and_flight_phase(self) -> bool:
        """Check kill switch and update flight phase."""
        if not self.offboard_mode_rc_switch_on:
            self.get_logger().warning(
                f"\nOffboard Callback: RC Flight Mode Channel {self.mode_channel} Switch Not Set to Offboard")
            self.offboard_setpoint_counter = 0
            return False

        self.program_time = time.time() - self.T0
        self.flight_phase = self.get_phase()

        self.get_logger().warn(
            f"\n[{self.program_time:.2f}s] In {self.flight_phase.name} phase for the next "
            f"{self.time_before_next_phase(self.flight_phase):.2f}s", throttle_duration_sec=0.5)

        return True

    def get_offboard_health(self) -> bool:
        """Check if vehicle is in offboard mode, armed, and has odometry data."""
        healthy = True

        if not self.in_offboard_mode:
            self.get_logger().warning("Vehicle is NOT in OFFBOARD mode.")
            healthy = False

        if not self.armed:
            self.get_logger().warning("Vehicle is NOT ARMED.")
            healthy = False

        if not self.mocap_initialized:
            self.get_logger().warning("Odometry is NOT received.")
            healthy = False

        return healthy

    def offboard_mode_timer_callback(self) -> None:
        """10Hz timer for offboard mode management."""
        if not self.killswitch_and_flight_phase():
            return

        if self.offboard_setpoint_counter == 10:
            engage_offboard_mode(self)
            arm(self)
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.flight_phase is FlightPhase.HOVER:
            publish_offboard_control_heartbeat_signal_position(self)

        elif self.flight_phase is FlightPhase.CUSTOM:
            publish_offboard_control_heartbeat_signal_bodyrate(self)

        elif self.flight_phase is FlightPhase.RETURN:
            publish_offboard_control_heartbeat_signal_position(self)

        elif self.flight_phase is FlightPhase.LAND:
            publish_offboard_control_heartbeat_signal_position(self)

        else:
            raise ValueError("Unknown flight phase")

    def data_log_timer_callback(self) -> None:
        """Callback function for the data logging timer."""
        if self.flight_phase is not FlightPhase.CUSTOM:
            return

        self.program_time_logtype.append(self.program_time)
        self.trajectory_time_logtype.append(self.trajectory_time)
        self.reference_time_logtype.append(self.reference_time)
        self.comptime_logtype.append(self.compute_time)

        self.x_logtype.append(self.x)
        self.y_logtype.append(self.y)
        self.z_logtype.append(self.z)
        self.yaw_logtype.append(self.yaw)

        self.xref_logtype.append(self.reff[0, 0])
        self.yref_logtype.append(self.reff[0, 1])
        self.zref_logtype.append(self.reff[0, 2])
        self.yawref_logtype.append(self.reff[0, -1])

        self.throttle_input_logtype.append(self.normalized_input[0])
        self.p_input_logtype.append(self.normalized_input[1])
        self.q_input_logtype.append(self.normalized_input[2])
        self.r_input_logtype.append(self.normalized_input[3])

    # --- Setpoint Publisher Functions --- #
    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish the position setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z, yaw]}", throttle_duration_sec=1.0)

    def publish_rates_setpoint(self, thrust: float, roll: float, pitch: float, yaw: float) -> None:
        """Publish the thrust and body rate setpoint."""
        msg = VehicleRatesSetpoint()
        msg.roll = float(roll)
        msg.pitch = float(pitch)
        msg.yaw = float(yaw)
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -1 * float(thrust)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.rates_setpoint_publisher.publish(msg)

        self.get_logger().info(f"Publishing rates setpoints [thrust, r,p,y]: {[thrust, roll, pitch, yaw]}",
                               throttle_duration_sec=1.0)

    def publish_control_timer_callback(self) -> None:
        throttle_val = 0.3
        if self.in_land_mode:
            self.get_logger().info("In land mode...")
            if abs(self.z) < 0.71 if self.sim else abs(self.z) < 0.64:
                self.get_logger().info("Landed, disarming...")
                disarm(self)
                exit(0)

        if not self.killswitch_and_flight_phase():
            return
        if not self.get_offboard_health():
            return

        if self.flight_phase is FlightPhase.HOVER:
            hover_pose = [0., 0., -self.HOVER_HEIGHT, 0.]
            self.publish_position_setpoint(*hover_pose)

        elif self.flight_phase is FlightPhase.CUSTOM:
            if self.buffer_valid:
                current_control = self.control_buffer[self.buffer_index, :]

                new_force = float(current_control[0])
                new_throttle = float(self.platform.get_throttle_from_force(new_force))
                new_roll_rate = float(current_control[1])
                new_pitch_rate = float(current_control[2])
                new_yaw_rate = float(current_control[3])
                self.normalized_input = [new_throttle, new_roll_rate, new_pitch_rate, new_yaw_rate]
                self.get_logger().warning(
                    f"Publishing buffer[{self.buffer_index}]: [thrust, r, p, y] = {current_control}",
                    throttle_duration_sec=throttle_val)

                self.publish_rates_setpoint(*self.normalized_input)
                self.buffer_index = min(self.buffer_index + 1, len(self.control_buffer) - 1)

        elif self.flight_phase is FlightPhase.RETURN:
            hover_pose = [0., 0., -self.HOVER_HEIGHT, 0.]
            self.publish_position_setpoint(*hover_pose)

        elif self.flight_phase is FlightPhase.LAND:
            land_pose = [0., 0., -self.LAND_HEIGHT, 0.]
            self.publish_position_setpoint(*land_pose)
            if abs(self.z) < 0.64:
                land(self)

    def compute_control_timer_callback(self) -> None:
        if not self.killswitch_and_flight_phase():
            return
        if not self.get_offboard_health():
            return
        if self.get_phase() is not FlightPhase.CUSTOM:
            return

        throttle_val = 0.3

        if not self.trajectory_started:
            self.trajectory_T0 = time.time()
            self.trajectory_time = 0.0
            self.trajectory_started = True

        self.trajectory_time = time.time() - self.trajectory_T0
        self.reference_time = self.trajectory_time + self.T_LOOKAHEAD
        self.get_logger().warning(
            f"\nTrajectory time: {self.trajectory_time:.2f}s, Reference time: {self.reference_time:.2f}s",
            throttle_duration_sec=throttle_val)

        ref, ref_dot = self.generate_ref_trajectory(self.ref_type)
        self.reff = np.hstack((ref[:, 0:3], ref_dot[:, 0:3], np.zeros((ref_dot.shape[0], 2)), ref[:, -1:]))
        

        self.get_logger().warning(f"\nCurrent state: {self.nmpc_state}", throttle_duration_sec=throttle_val)
        self.get_logger().warning(f"\nCurrent ref: {self.reff[0, :]}", throttle_duration_sec=throttle_val)
        self.get_logger().warning(f"\nCurrent error: {self.nmpc_state - self.reff[0, :]}",
                                  throttle_duration_sec=throttle_val)

        t0 = time.time()
        self.controller_handler()
        self.compute_time = time.time() - t0
        self.get_logger().warning(f"\nMPC Solver Status: {self.status}", throttle_duration_sec=throttle_val)
        self.get_logger().warning(
            f"\nControl computation time: {self.compute_time:.4f}s, Good for {1.0 / self.compute_time:.2f} Hz",
            throttle_duration_sec=throttle_val)

        self.new_input = self.new_inputs[0, :].flatten()
        self.get_logger().warning(f"\nNew control input: {self.new_input}", throttle_duration_sec=throttle_val)

        self.control_buffer = self.new_inputs
        self.buffer_index = 0
        self.buffer_valid = True

    def controller_handler(self):
        """Wrapper for controller computation."""
        if self.pyjoules_on:
            with EnergyContext(handler=self.csv_handler, domains=[RaplPackageDomain(0)]):  # type: ignore
                self.controller()
        else:
            self.controller()

    def controller(self):
        """Compute control input using error-based NMPC."""
        u_nmpc, _, status = self.mpc_solver.solve_mpc_control(
            self.nmpc_state, self.reff, self.last_input, nx=9, nu=4, verbose=False)
        self.new_inputs = u_nmpc
        self.status = status

    def generate_ref_trajectory(self, traj_type: TrajectoryType, **ctx_overrides):
        """Generate reference trajectory."""
        ctx_dict = {
            'sim': self.sim,
            'hover_mode': ctx_overrides.get('hover_mode', self.hover_mode),
            'spin': ctx_overrides.get('spin', self.spin),
            'double_speed': ctx_overrides.get('double_speed', True),
            'short': ctx_overrides.get('short', self.short),
        }
        ctx = TrajContext(**ctx_dict)
        traj_func = TRAJ_REGISTRY[traj_type]

        return generate_reference_trajectory(
            traj_func=traj_func,
            t_start=self.reference_time,
            horizon=self.horizon,
            num_steps=self.num_steps,
            ctx=ctx
        )
