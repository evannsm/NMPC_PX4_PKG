# First non-pyjoules except for helix spin, then pyjoules except for helix spin.
# Separately after we'll do helix spin with and without pyjoules.

## Part 1: All Non-PyJoules Except Helix Spin

```bash
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory hover      --double-speed --hover-mode 1 --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory yaw_only   --double-speed --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory circle_horz --double-speed --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory circle_horz --double-speed --spin --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory circle_vert --double-speed --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_horz   --double-speed --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_vert   --double-speed --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_vert   --double-speed --short --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory helix       --double-speed --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory sawtooth    --double-speed --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory triangle    --double-speed --log
```

## Part 2: All PyJoules Except Helix Spin

```bash
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory hover      --double-speed --hover-mode 1 --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory yaw_only   --double-speed --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory circle_horz --double-speed --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory circle_horz --double-speed --spin --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory circle_vert --double-speed --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_horz   --double-speed --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_vert   --double-speed --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory fig8_vert   --double-speed --short --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory helix       --double-speed --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory sawtooth    --double-speed --pyjoules --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory triangle    --double-speed --pyjoules --log
```

## Part 3: Helix Spin with and without PyJoules

```bash
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory helix --double-speed --spin --log
ros2 run nmpc_acados_euler_err run_node --platform sim --trajectory helix --double-speed --spin --pyjoules --log
---

