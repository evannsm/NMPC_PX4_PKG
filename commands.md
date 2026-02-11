# NMPC Acados Euler Error Commands

NMPC controller using an **Euler-angle state representation** with a **wrapped yaw error** in the cost function.

---

## Build Package

```bash
cd ~/ws_clean_traj
colcon build --packages-select nmpc_px4_pkg
source install/setup.bash
```

---

## Run Commands

### Simulation

```bash
# Hover (requires --hover-mode)
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory hover --hover-mode 1

# Circle Horizontal
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory circle_horz
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory circle_horz --double-speed
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory circle_horz --spin
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory circle_horz --double-speed --spin

# Circle Vertical
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory circle_vert
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory circle_vert --double-speed

# Figure-8 Horizontal
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory fig8_horz
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory fig8_horz --double-speed

# Figure-8 Vertical
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory fig8_vert
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory fig8_vert --double-speed
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory fig8_vert --short
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory fig8_vert --double-speed --short

# Helix
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory helix
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory helix --double-speed
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory helix --spin
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory helix --double-speed --spin

# Yaw Only
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory yaw_only
ros2 run nmpc_px4_pkg run_node --platform sim --trajectory yaw_only --double-speed
```

---

### Hardware

```bash
# Hover (modes 1–4 only on hardware)
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory hover --hover-mode 1
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory hover --hover-mode 2
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory hover --hover-mode 3
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory hover --hover-mode 4

# Circle Horizontal
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory circle_horz
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory circle_horz --double-speed

# Helix
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory helix
ros2 run nmpc_px4_pkg run_node --platform hw --trajectory helix --double-speed
```

---

## With Logging

Add `--log` to enable data logging.  
If no filename is provided, a log name is **auto-generated** from the configuration.

```bash
# Auto-generated filename:
# sim_nmpc_px4_pkg_helix_2x_spin.csv
ros2 run nmpc_px4_pkg run_node \
  --platform sim \
  --trajectory helix \
  --double-speed \
  --spin \
  --log
```

### Custom Log Filename

```bash
ros2 run nmpc_px4_pkg run_node \
  --platform sim \
  --trajectory helix \
  --log \
  --log-file my_experiment
```

`.csv` is automatically appended if not provided.

---

## With PyJoules Energy Monitoring

```bash
ros2 run nmpc_px4_pkg run_node \
  --platform sim \
  --trajectory helix \
  --double-speed \
  --log \
  --pyjoules
```

---

## Arguments Reference

| Argument         | Required | Values                                                                               | Description                        |
| ---------------- | -------- | ------------------------------------------------------------------------------------ | ---------------------------------- |
| `--platform`     | Yes      | `sim`, `hw`                                                                          | Platform type                      |
| `--trajectory`   | Yes      | `hover`, `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix` | Trajectory type                    |
| `--hover-mode`   | If hover | `1–8` (sim), `1–4` (hw)                                                              | Hover position                     |
| `--double-speed` | No       | flag                                                                                 | Use 2× trajectory speed            |
| `--short`        | No       | flag                                                                                 | Short `fig8_vert` variant          |
| `--spin`         | No       | flag                                                                                 | Enable yaw rotation                |
| `--log`          | No       | flag                                                                                 | Enable data logging                |
| `--log-file`     | No       | string                                                                               | Custom log filename (no extension) |
| `--pyjoules`     | No       | flag                                                                                 | Enable energy monitoring           |

---

## Key Features

- **Wrapped yaw error** using `atan2(sin(Δψ), cos(Δψ))`
- **Error-based MPC cost** with references passed via parameters
- **Stage-wise reference parameters** at each MPC stage