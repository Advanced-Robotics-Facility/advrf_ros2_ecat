# EtherCAT bridge

`EC_CFG` selects the client YAML. Device startup is owned by the EtherCAT client.

## TX activation

TX starts **inactive**. Feedback (`rx/motors`, `rx/joint_states`) stays enabled.
`active_on_first_msg` defaults to `false`; `read_only` has been removed.

```bash
ros2 launch advrf_ros2_miranda_bringup bridge.launch.py active_on_first_msg:=true
# Or run the bridge directly:
ros2 run advrf_ros2_ecat bridge_ecat_client_node --ros-args -p active_on_first_msg:=true
```

With `true`, the first valid motor command activates TX and starts the watchdog;
invalid messages do not activate it. The watchdog is off while waiting.
Automatic activation is one-shot: an explicit `set_active(false)` cancels it,
including before the first command. Use the service to reactivate afterward.
With `false`, activation is always explicit through the service or joystick A.

```bash
ros2 service call /advrf/miranda/ecat/set_active std_srvs/srv/SetBool '{data: true}'
ros2 service call /advrf/miranda/ecat/set_active std_srvs/srv/SetBool '{data: false}'
ros2 topic echo /advrf/miranda/ecat/active
```

- `true`: allow ROS motor commands and arm the watchdog immediately.
- `false`: block command forwarding, discard queued writes and disarm the watchdog.
- `active`: reliable, transient-local `std_msgs/Bool`; authoritative state for clients.
- `command_timeout_ms`: 500 ms by default, refreshed only by valid commands while active.
- Timeout: set active false, stop the writer/devices, latch FAULT. Restart the bridge before reactivation.
- Repeating `true` while already active does not refresh the watchdog.

Disabling TX does not power off motors or clear the client's last applied reference.
The existing hardware/client startup and fault STOP remain separate from TX activation.
Diagnostics on `/diagnostics` report `active`, `watchdog_armed`, and `mode`.
