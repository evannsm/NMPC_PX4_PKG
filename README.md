# nmpc_acados_euler_err

A ROS 2 Nonlinear Model Predictive Controller (NMPC) for quadrotors using the [Acados](https://docs.acados.org/) solver. Formulates the tracking problem with an error-based cost in Euler angle representation and uses `atan2`-based yaw wrapping for correct angular error computation.

## Approach

This controller solves a finite-horizon optimal control problem at every timestep:

1. **Model** — 9D state `[x, y, z, vx, vy, vz, roll, pitch, yaw]` with 4D input `[thrust, p, q, r]`
2. **Error-based cost** — the stage cost penalizes `[position_err, velocity_err, euler_err, input_err]` (13D); the terminal cost drops the input term (9D)
3. **Wrapped yaw error** — uses `atan2(sin(yaw - yaw_ref), cos(yaw - yaw_ref))` to avoid discontinuities at +/-pi
4. **Acados solver** — generates and compiles C code for the QP sub-problems, enabling real-time MPC

## Key Features

- **Acados C-code generation** — solver is compiled once and cached for fast startup on subsequent runs
- **Error-state cost formulation** — references are passed as stage-wise parameters, not embedded in the cost
- **Nonlinear least squares** — cost type is `NONLINEAR_LS` with configurable weight matrices
- **Input constraints** — hard bounds on thrust `[0, 27] N` and body rates `[-0.8, 0.8] rad/s`
- **PX4 integration** — publishes attitude setpoints and offboard commands via `px4_msgs`
- **Structured logging** — optional CSV logging via ROS2Logger

## Cost Weights

**Stage cost (13D):**

| Component | Weight | Dimension |
|-----------|--------|-----------|
| Position error | `2e3` | 3 |
| Velocity error | `2e1` | 3 |
| Roll/pitch error | `2e1` | 2 |
| Yaw error | `2e2` | 1 |
| Thrust | `1e1` | 1 |
| Body rates (p, q) | `1e3` | 2 |
| Yaw rate (r) | `1e2` | 1 |

**Terminal cost (9D):** same as stage cost without input terms (position `1e3`, velocity `1e1`, roll/pitch `1e1`, yaw `1e2`).

## Usage

```bash
source install/setup.bash

# Fly a figure-8 in simulation
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_horz

# Hardware flight with logging
ros2 run nmpc_acados_euler_err run_node --platform hw --trajectory helix --log
```

### CLI Options

| Flag | Description |
|------|-------------|
| `--platform {sim,hw}` | Target platform (required) |
| `--trajectory {hover,yaw_only,circle_horz,...}` | Trajectory type (required) |
| `--hover-mode {1..8}` | Hover sub-mode (1-4 for hardware) |
| `--log` | Enable CSV data logging |
| `--log-file NAME` | Custom log filename |
| `--double-speed` | 2x trajectory speed |
| `--short` | Short variant (fig8_vert) |
| `--spin` | Enable yaw rotation |
| `--flight-period SEC` | Custom flight duration |

## Dependencies

- [quad_trajectories](https://github.com/evannsm/quad_trajectories) — trajectory definitions
- [quad_platforms](https://github.com/evannsm/quad_platforms) — platform abstraction
- [ROS2Logger](https://github.com/evannsm/ROS2Logger) — experiment logging
- [px4_msgs](https://github.com/PX4/px4_msgs) — PX4 ROS 2 message definitions
- [Acados](https://docs.acados.org/) and `acados_template`
- SciPy

## Package Structure

```
nmpc_acados_euler_err/
├── nmpc_acados_euler_err/
│   ├── run_node.py              # CLI entry point and argument parsing
│   └── ros2px4_node.py          # ROS 2 node (subscriptions, publishers, control loop)
└── nmpc_acados_euler_err_utils/
    ├── controller/
    │   └── nmpc/
    │       ├── generate_nmpc.py # NMPC problem formulation and C-code generation
    │       └── acados_model.py  # Quadrotor Euler dynamics model for Acados
    ├── px4_utils/               # PX4 interface and flight phase management
    ├── transformations/         # Yaw adjustment utilities
    ├── main_utils.py            # Helper functions
    └── jax_utils.py             # JAX configuration
```

## Installation

```bash
# Inside a ROS 2 workspace src/ directory
git clone git@github.com:evannsm/nmpc_acados_euler_err.git
cd .. && colcon build --symlink-install
```

> **Note:** Acados must be installed separately. See [Acados installation guide](https://docs.acados.org/installation/).

## License

MIT
