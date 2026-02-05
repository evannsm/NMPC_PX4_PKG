"""
CLI Entry Point for NMPC with Euler State and Wrapped Yaw Error
===============================================================

Run with: ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory helix --log test_log
"""

import rclpy
import traceback
import argparse
import os

from Logger import Logger  # type: ignore
from .ros2px4_node import OffboardControl

from nmpc_acados_euler_err_utils.main_utils import BANNER
from quad_platforms import PlatformType
from quad_trajectories import TrajectoryType
from pyJoules.handler.csv_handler import CSVHandler


def create_parser():
    """Create and configure argument parser."""
    parser = argparse.ArgumentParser(
        description='NMPC Offboard Control with Euler State and Wrapped Yaw Error',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        """ + "==" * 60 + """
        Example usage:
        ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory hover --hover-mode 1 --log test_log
        ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory helix --double-speed --log helix_log
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

    # Optional flags
    parser.add_argument(
        '--pyjoules',
        action='store_true',
        help='Enable PyJoules energy monitoring'
    )

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
        '--log',
        type=str,
        required=True,
        help='Log file name without extension'
    )

    return parser


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


def main():
    """Main entry point for the executable."""
    parser = create_parser()
    args = parser.parse_args()
    validate_args(args, parser)

    platform = args.platform
    trajectory = args.trajectory
    hover_mode = args.hover_mode
    pyjoules = args.pyjoules
    double_speed = args.double_speed
    short = args.short
    spin = args.spin
    log_file = args.log
    base_path = os.path.dirname(os.path.abspath(__file__))

    # Print configuration
    print("\n" + "=" * 60)
    print("NMPC Euler Error Offboard Control Configuration")
    print("=" * 60)
    print(f"Controller:    NMPC with Wrapped Yaw Error (Euler State)")
    print(f"Platform:      {platform.value.upper()}")
    print(f"Trajectory:    {trajectory.value.upper()}")
    print(f"Hover Mode:    {hover_mode if hover_mode is not None else 'N/A'}")
    print(f"PyJoules:      {'Enabled' if pyjoules else 'Disabled'}")
    print(f"Speed:         {'Double (2x)' if double_speed else 'Regular (1x)'}")
    print(f"Short:         {'Enabled (fig8_vert)' if short else 'Disabled'}")
    print(f"Spin:          {'Enabled (circle_horz, helix)' if spin else 'Disabled'}")
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
        csv_handler=CSVHandler(log_file, base_path) if pyjoules else None
    )

    logger = None

    def shutdown_logging(*args):
        print("\nShutting down, triggering logging...")
        if logger:
            logger.log(offboard_control_node)
        offboard_control_node.destroy_node()
        rclpy.shutdown()

    try:
        print(f"\nInitializing Offboard Control Node")
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
        print("Saving log data...")
        shutdown_logging()
        print("\nNode shut down.")


if __name__ == '__main__':
    main()
