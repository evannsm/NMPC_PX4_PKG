# nmpc_acados_euler_err Commands

NMPC controller with Euler state representation and wrapped yaw error cost.

## Build

```bash
cd ~/ws_clean_traj
colcon build --packages-select nmpc_acados_euler_err
source install/setup.bash
```

## Run

### Simulation (Hover)
```bash
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory hover --hover-mode 1 --log hover_test
```

### Simulation (Helix with double speed)
```bash
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory helix --double-speed --log helix_test
```

### Simulation (Figure-8 Vertical)
```bash
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_vert --double-speed --log fig8_test
```

### Hardware
```bash
ros2 run nmpc_acados_euler_err run_node --platform hw --trajectory helix --double-speed --log hw_helix
```

## Available Options

- `--platform`: `sim` or `hw`
- `--trajectory`: `hover`, `helix`, `fig8_vert`, `fig8_horz`, `circle_horz`, `circle_vert`
- `--hover-mode`: 1-8 (only for hover trajectory)
- `--double-speed`: Use 2x speed for trajectory
- `--short`: Use short variant (fig8_vert only)
- `--spin`: Enable spin (circle_horz, helix)
- `--pyjoules`: Enable energy monitoring
- `--log`: Log file name (required)

## Key Features

- **Wrapped yaw error**: Uses `atan2(sin(yaw_diff), cos(yaw_diff))` for proper angle wrapping
- **Error-based cost**: Cost expression outputs errors, yref is zeros
- **Stage-wise parameters**: References passed via parameters at each MPC stage
