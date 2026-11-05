# EtherCAT bridge

`EC_CFG` selects the EtherCAT client configuration, including the robot ID map
and motor settings. ROS topic names are selected independently by the node
namespace. The default namespace is `/advrf/bench/ecat`.

For Miranda, keep `repl -d` running and start the bridge with:

```bash
ros2 run advrf_ros2_ecat bridge_ecat_client_node --ros-args \
  -r __ns:=/advrf/miranda/ecat -p period_ms:=4
```

The bridge publishes `/advrf/miranda/ecat/rx/motors` and
`/advrf/miranda/ecat/rx/joint_states`, and accepts commands on
`/advrf/miranda/ecat/tx/motors`. Diagnostics are published on `/diagnostics`.
Use `-p read_only:=true` to omit the motor command subscription. Motors are
started only after the first valid command; after that, the command watchdog
stops them if no valid command arrives within `command_timeout_ms` (default 500).

The Miranda `debug.launch.py` launch file already selects the Miranda namespace.

The static EtherCAT libraries bundle spdlog separately from ROS. The bridge
linker hides symbols from static archives to prevent the bundled version from
overriding ROS's shared library. Without this isolation, the Xenomai build can
print `[*** LOG ERROR ...] ... Operation not permitted` during periodic ROS log
flushing, even when no EtherCAT client has been started. Rebuild and source the
workspace after updating the linker settings.
