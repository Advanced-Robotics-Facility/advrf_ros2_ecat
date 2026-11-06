# EtherCAT bridge

`EC_CFG` selects the client YAML. Device startup is owned by the EtherCAT client.

## TX activation

TX starts **inactive**. Feedback (`rx/motors`, `rx/joint_states`) stays enabled.

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
- `read_only:=true` rejects activation.

Disabling TX does not power off motors or clear the client's last applied reference.
The existing hardware/client startup and fault STOP remain separate from TX activation.
Diagnostics on `/diagnostics` report `active`, `watchdog_armed`, and `mode`.
