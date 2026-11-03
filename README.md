# EtherCAT diagnostics and command watchdog

The node publishes `diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics`:

- `PIPE OK - waiting TX to start`: no valid motor command received yet.
- `PIPE OK - TX running`: valid commands are arriving.
- `PIPE OK (read only)`: command subscription disabled.
- `FAULT`: command timeout, stopped EtherCAT client, or stopped writer.

`command_timeout_ms` is a positive startup parameter (default: 500 ms).
The watchdog uses a monotonic clock and arms on the first valid command on
`tx/motors`. Invalid commands do not refresh it. Before that first command,
there is no command timeout, including with `auto_start=true`.

On timeout, the node joins the writer, requests `stop_devices()`, rejects
further commands, and keeps publishing the latched fault. Restart the node
to clear the fault. A failed STOP request is reported in the diagnostic.
The watchdog runs in the main loop: detection is subject to its scheduling
and any blocking EtherCAT calls; this is not an independent hardware watchdog.

Example parameter override:

```bash
EC_CFG=/path/ec_cfg.yaml ros2 run advrf_ros2_ecat bridge_ecat_client_node --ros-args -p command_timeout_ms:=500
```

## rqt Robot Monitor

Robot Monitor consumes `/diagnostics_agg`. If an aggregator already exists,
add the EtherCAT analyzer from `config/diagnostics.yaml` to its configuration.
Otherwise, with `diagnostic_aggregator` and `rqt_robot_monitor` installed, run:

```bash
ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file "$(ros2 pkg prefix advrf_ros2_ecat)/share/advrf_ros2_ecat/config/diagnostics.yaml"
ros2 run rqt_robot_monitor rqt_robot_monitor
```

Use the same ROS domain as the EtherCAT node. The EtherCAT entry exposes
mode, pipe state, watchdog activation, timeout, last valid command age,
and motor activation state.
