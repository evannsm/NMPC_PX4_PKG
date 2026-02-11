"""Entry point for the NMPC Acados Fixed Euler Error control ROS2 node."""

import rclpy
import traceback
import argparse
import os

from Logger import Logger  # type: ignore
from .ros2px4_node import OffboardControl

from quad_platforms import PlatformType
from quad_trajectories import TrajectoryType
from pyJoules.handler.csv_handler import CSVHandler

def create_parser():
    """Create and configure argument parser.

    Args:
        platform: 'sim' or 'hw'
        trajectory: Trajectory name (e.g., 'circle_horz', 'fig8_vert', etc.)
        log: Enable data logging (auto-generates filename if --log-file not provided)
        log_file: Custom log file name (optional, overrides auto-generated name)
        pyjoules: Enable PyJoules energy monitoring
        double_speed: Use double speed for trajectories
        short: Use short variant for fig8_vert trajectory
        spin: Enable spin for circle_horz and helix trajectories
    """
    parser = argparse.ArgumentParser(
        description='NMPC Offboard Control with Euler State and Wrapped Yaw Error',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        """ + "==" * 60 + """
        Example usage:
        # Auto-generated log filename:
        ros2 run nmpc_px4_pkg run_node --platform sim --trajectory helix --double-speed --spin --log
        # -> logs to: sim_nmpc_px4_pkg_helix_2x_spin.csv

        # Custom log filename:
        ros2 run nmpc_px4_pkg run_node --platform sim --trajectory helix --log --log-file my_custom_log
        """ + "==" * 60 + """
        """
    )

    # Required arguments
    parser.add_argument(
        "--platform",
        type=PlatformType,
        choices=list(PlatformType),
        required=True,
        help="Platform type to use. Options: " + ", ".join(e.value for e in PlatformType) + ".",
    )

    parser.add_argument(
        '--trajectory',
        type=TrajectoryType,
        choices=list(TrajectoryType),
        required=True,
        help="Trajectory type to execute. Options: " + ", ".join(e.value for e in TrajectoryType) + ".",
    )

    parser.add_argument(
        "--hover-mode",
        type=int,
        choices=range(1, 9),
        help="Hover mode (required when --trajectory=hover). On hardware only 1-4 are allowed.",
    )

    # Logging flags (match NR behavior)
    parser.add_argument(
        '--log',
        action='store_true',
        help='Enable data logging. Auto-generates filename based on config unless --log-file is provided.'
    )

    parser.add_argument(
        '--log-file',
        type=str,
        default=None,
        help='Custom log file name (without extension). Overrides auto-generated name when --log is used.'
    )

    parser.add_argument(
        '--pyjoules',
        action='store_true',
        help='Enable PyJoules energy monitoring (separate from --log)'
    )

    # Trajectory modifier flags
    parser.add_argument(
        '--double-speed',
        action='store_true',
        help='Use double speed (2x) for trajectories'
    )

    parser.add_argument(
        '--short',
        action='store_true',
        help='Use short variant for fig8_vert trajectory'
    )

    parser.add_argument(
        '--spin',
        action='store_true',
        help='Enable spin for circle_horz and helix trajectories'
    )

    parser.add_argument(
        '--flight-period',
        type=float,
        default=None,
        help='Set custom flight period in seconds (default: 30s)'
    )


    return parser


def ensure_csv(filename: str) -> str:
    """Return filename that ends with exactly one '.csv' (case-insensitive)."""
    filename = filename.strip()
    if filename.lower().endswith(".csv"):
        return filename[:-4] + ".csv"   # normalize casing to '.csv'
    return filename + ".csv"


def generate_log_filename(args) -> str:
    """Generate auto log filename based on configuration.

    Format: {platform}_nmpc_px4_pkg_{trajectory}_{speed}[_{short}][_{spin}]

    Examples:
        sim_nmpc_px4_pkg_helix_2x_spin.csv
        sim_nmpc_px4_pkg_circle_horz_1x.csv
        hw_nmpc_px4_pkg_fig8_vert_2x_short.csv
    """
    parts = []
    parts.append(args.platform.value)               # 'sim' or 'hw'
    parts.append("nmpc_px4_pkg")           # controller id
    parts.append(args.trajectory.value)             # e.g., 'helix'
    parts.append("2x" if args.double_speed else "1x")

    if args.short:
        parts.append("short")
    if args.spin:
        parts.append("spin")

    return "_".join(parts)


def validate_args(args, parser: argparse.ArgumentParser) -> None:
    """Validate command-line arguments."""
    if args.trajectory == TrajectoryType.HOVER:
        if args.hover_mode is None:
            parser.error("--hover-mode is required when --trajectory=hover")
        if args.platform == PlatformType.HARDWARE and args.hover_mode not in range(1, 5):
            parser.error("--hover-mode must be 1-4 for --platform=hw")
        if args.platform == PlatformType.SIM and args.hover_mode not in range(1, 9):
            parser.error("--hover-mode must be 1-8 for --platform=sim")
    else:
        if args.hover_mode is not None:
            parser.error("--hover-mode is only valid when --trajectory=hover")

    # Match NR behavior: disallow --log-file unless --log is enabled
    if args.log_file is not None and not args.log:
        parser.error("--log-file requires --log to be enabled")


def main():
    """Main entry point for the executable."""
    parser = create_parser()
    args = parser.parse_args()
    validate_args(args, parser)

    platform = args.platform
    trajectory = args.trajectory
    hover_mode = args.hover_mode
    logging_enabled = args.log
    pyjoules = args.pyjoules
    double_speed = args.double_speed
    short = args.short
    spin = args.spin
    flight_period = args.flight_period
    base_path = os.path.dirname(os.path.abspath(__file__))

    # Determine log filename
    if logging_enabled:
        if args.log_file is not None:
            log_file_stem = args.log_file
        else:
            log_file_stem = generate_log_filename(args)

        log_file = ensure_csv(log_file_stem)  # ALWAYS ends with .csv
    else:
        log_file = None

    # Print configuration
    print("\n" + "=" * 60)
    print("NMPC Euler Error Offboard Control Configuration")
    print("=" * 60)
    print("Controller:    NMPC with Wrapped Yaw Error (Euler State)")
    print(f"Platform:      {platform.value.upper()}")
    print(f"Trajectory:    {trajectory.value.upper()}")
    print(f"Hover Mode:    {hover_mode if hover_mode is not None else 'N/A'}")
    print(f"PyJoules:      {'Enabled' if pyjoules else 'Disabled'}")
    print(f"Speed:         {'Double (2x)' if double_speed else 'Regular (1x)'}")
    print(f"Short:         {'Enabled (fig8_vert)' if short else 'Disabled'}")
    print(f"Spin:          {'Enabled (circle_horz, helix)' if spin else 'Disabled'}")
    print(f"Flight Period: {flight_period if flight_period is not None else 60.0 if platform == PlatformType.HARDWARE else 30.0} seconds")
    print(f"Data Logging:  {'Enabled' if logging_enabled else 'Disabled'}")
    
    if logging_enabled:
        print(f"Log File:      {log_file}")
    print("=" * 60 + "\n")

    rclpy.init(args=None)
    offboard_control_node = OffboardControl(
        platform_type=platform,
        trajectory=trajectory,
        hover_mode=hover_mode,
        double_speed=double_speed,
        short=short,
        spin=spin,
        pyjoules=pyjoules,
        csv_handler=CSVHandler(log_file, base_path) if pyjoules and log_file else None,
        logging_enabled=logging_enabled,
        flight_period_=flight_period
    )

    logger = None

    def shutdown_logging(*args):
        print("\nShutting down, triggering logging...")
        if logger and logging_enabled:
            logger.log(offboard_control_node)
        offboard_control_node.destroy_node()
        rclpy.shutdown()

    try:
        print("\nInitializing Offboard Control Node")
        if logging_enabled:
            logger = Logger(log_file, base_path)
        rclpy.spin(offboard_control_node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt (Ctrl+C)")
    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
    finally:
        if pyjoules and offboard_control_node.csv_handler:
            print("Saving PyJoules energy data...")
            offboard_control_node.csv_handler.save_data()

        if logging_enabled:
            print("Saving log data...")
        shutdown_logging()
        print("\nNode shut down.")


if __name__ == '__main__':
    main()
