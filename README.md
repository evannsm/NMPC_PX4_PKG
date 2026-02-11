# Nonlinear NMPC for PX4-ROS2 Deployment

A ROS 2 Nonlinear Model Predictive Controller (NMPC) for quadrotors using the [Acados](https://docs.acados.org/) solver. Formulates the tracking problem with an error-based cost in Euler angle representation and uses `atan2`-based yaw wrapping for correct angular error computation.

This package was created during my PhD originally as a basis of comparison with the well-established NMPC technique in order to make useful comparisons against novel control strategies (namely, Newton-Raphson Flow) developed at Georgia Tech's FACTSLab. We have compared this against the Newton-Raphson controller available in [`NRFlow_PX4_PKG`](https://github.com/evannsm/NRFlow_PX4_PKG).

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

| Component         | Weight | Dimension |
| ----------------- | ------ | --------- |
| Position error    | `2e3`  | 3         |
| Velocity error    | `2e1`  | 3         |
| Roll/pitch error  | `2e1`  | 2         |
| Yaw error         | `2e2`  | 1         |
| Thrust            | `1e1`  | 1         |
| Body rates (p, q) | `1e3`  | 2         |
| Yaw rate (r)      | `1e2`  | 1         |

**Terminal cost (9D):** same as stage cost without input terms (position `1e3`, velocity `1e1`, roll/pitch `1e1`, yaw `1e2`).

## Usage

```bash
source install/setup.bash

# Fly a figure-8 in simulation
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory fig8_horz

# Hardware flight with logging
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory helix --log
```

### CLI Options

| Flag                                            | Description                       |
| ----------------------------------------------- | --------------------------------- |
| `--platform {sim,hw}`                           | Target platform (required)        |
| `--trajectory {hover,yaw_only,circle_horz,...}` | Trajectory type (required)        |
| `--hover-mode {1..8}`                           | Hover sub-mode (1-4 for hardware) |
| `--log`                                         | Enable CSV data logging           |
| `--log-file NAME`                               | Custom log filename               |
| `--double-speed`                                | 2x trajectory speed               |
| `--short`                                       | Short variant (fig8_vert)         |
| `--spin`                                        | Enable yaw rotation               |
| `--flight-period SEC`                           | Custom flight duration            |

## Dependencies

- [quad_trajectories](https://github.com/evannsm/quad_trajectories) — trajectory definitions
- [quad_platforms](https://github.com/evannsm/quad_platforms) — platform abstraction
- [ROS2Logger](https://github.com/evannsm/ROS2Logger) — experiment logging
- [px4_msgs](https://github.com/PX4/px4_msgs) — PX4 ROS 2 message definitions
- [Acados](https://docs.acados.org/) and `acados_template`
- SciPy

## Package Structure

```
nmpc_px4_pkg/
├── nmpc_px4_pkg/
│   ├── run_node.py              # CLI entry point and argument parsing
│   └── ros2px4_node.py          # ROS 2 node (subscriptions, publishers, control loop)
└── nmpc_px4_pkg_utils/
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
git clone git@github.com:evannsm/nmpc_px4_pkg.git
cd .. && colcon build --symlink-install
```

## Acados Setup

Acados must be installed separately before building this package. Follow the steps below or the [official instructions](https://docs.acados.org/installation/index.html).

### 1) Install Acados

Clone and build from source:

```bash
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
```

```bash
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
# add more optional arguments e.g. -DACADOS_WITH_DAQP=ON
make install -j4
```

### 2) Install the Acados Python interface

Install the template as per the [Python interface instructions](https://docs.acados.org/python_interface/index.html):

```bash
pip install -e <acados_root>/interfaces/acados_template
```

Add these environment variables to your shell init (e.g., `~/.bashrc`), then `source ~/.bashrc`:

```bash
acados_root="your_acados_root"  # e.g. "/home/user/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$acados_root/lib"
export ACADOS_SOURCE_DIR="$acados_root"
```

### 3) Install t_renderer binaries

Required for rendering C code templates:

1. Download the correct binaries from the [t_renderer releases](https://github.com/acados/tera_renderer/releases/)
2. Place the binary in `<acados_root>/bin`
3. Rename it to `t_renderer` (e.g. `t_renderer-v0.2.0-linux-arm64` -> `t_renderer`)
4. Make it executable:

```bash
chmod +x <acados_root>/bin/t_renderer
```

### 4) Verify the installation

```bash
python3 <acados_root>/examples/acados_python/getting_started/minimal_example_ocp.py
```

If it runs and plots with no errors, you're done!

## Papers and Repositories

American Control Conference 2024 — [paper](https://coogan.ece.gatech.edu/papers/pdf/cuadrado2024tracking.pdf)
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_ACC2024)
| [FACTSLab repo](https://github.com/gtfactslab/MoralesCuadrado_Llanes_ACC2024)

Transactions on Control Systems Technology 2025 — [paper](https://arxiv.org/abs/2508.14185)
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)
| [FACTSLab repo](https://github.com/gtfactslab/Baird_MoralesCuadrado_TRO_2025)

Transactions on Robotics 2025
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)
| [FACTSLab repo](https://github.com/gtfactslab/MoralesCuadrado_Baird_TCST2025)

### Related Work

- [2025_NewtonRaphson_QuadrotorComplete](https://github.com/evannsm/2025_NewtonRaphson_QuadrotorComplete)
- [Blimp_SimHardware_NR_MPC_FBL_BodyOfWork2024](https://github.com/evannsm/Blimp_SimHardware_NR_MPC_FBL_BodyOfWork2024)

## License

MIT
