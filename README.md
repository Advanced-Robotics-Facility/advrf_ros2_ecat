# advrf_ros2_ecat
ros version: jazzy

## Quick Use

Prerequisites: EtherCAT master/backend running; workspace sourced.

```bash
export EC_CFG=/absolute/path/to/ec_cfg.yaml
ros2 run advrf_ros2_ecat bridge_ecat_client_node
```

Launch via `advrf_ros2_bringup`:

```bash
export EC_CFG=/absolute/path/to/ec_cfg.yaml
ros2 launch advrf_ros2_bringup hardware.launch.py \
  bridge_config:=/absolute/path/to/advrf_bridge.yaml \
  namespace:=/advrf/miranda
```

Enable TX (standalone default namespace):

```bash
ros2 service call /advrf/bench/ecat/set_active_tx std_srvs/srv/SetBool '{data: true}'
```

## Edit Config

- `EC_CFG`: EtherCAT client YAML; network, client period, robot ID map, control modes, gains.
- `bridge_config`: explicit ROS parameter YAML path; loaded by `hardware.launch.py`.
- Standalone overrides: `--ros-args -p period_ms:=4 -p command_timeout_ms:=500`.
- Standalone namespace: `--ros-args -r __ns:=/advrf/miranda/ecat`.
- Launch: single `namespace` argument; EtherCAT endpoints under `<namespace>/ecat`.
- `config/diagnostics.yaml`: optional `diagnostic_aggregator` configuration.

## Nodes Details

Node: `bridge_ecat_client_node` / ROS 2 <-> EtherCAT motor bridge

Parameters (startup only):

| Parameter | Default | Values / purpose |
| --- | --- | --- |
| `period_ms` | `4` | Positive integer; ROS loop period |
| `active_tx_on_first_msg` | `false` | Auto-enable TX on first valid command |
| `command_timeout_ms` | `500` | Positive integer; command watchdog timeout |
| `qos.state.depth` | `1` | `1..100000` |
| `qos.state.reliability` | `reliable` | `reliable`, `best_effort` |
| `qos.state.durability` | `volatile` | `volatile`, `transient_local` |
| `qos.command.depth` | `1` | `1..100000` |
| `qos.command.reliability` | `best_effort` | `reliable`, `best_effort` |
| `qos.command.durability` | `volatile` | `volatile`, `transient_local` |

Bringup YAML override: `qos.state.depth: 10`.

Topics (relative to node namespace, except `/diagnostics`):

| Topic | Direction | Type | QoS |
| --- | --- | --- | --- |
| `rx/motors` | Publish | `advrf_ros2_interfaces/msg/MotorRxPdo` | `qos.state.*` |
| `rx/joint_states` | Publish | `sensor_msgs/msg/JointState` | `qos.state.*` |
| `tx/motors` | Subscribe | `advrf_ros2_interfaces/msg/MotorTxPdo` | `qos.command.*` |
| `active` | Publish | `std_msgs/msg/Bool` | Reliable, transient local, depth 1 |
| `/diagnostics` | Publish | `diagnostic_msgs/msg/DiagnosticArray` | Reliable, volatile, depth 10 |

Services:

- `set_active_tx` / `std_srvs/srv/SetBool`: enable/disable TX; disable disarms watchdog.

Actions: none.

Command handling:

- Watchdog: armed after first valid command; timeout → motor STOP attempt and node shutdown.
- `active`: `true` while TX enabled or automatic activation pending.
- Batch: 1–128 entries; unique configured EtherCAT IDs; finite references/gains; partial batches supported.
- Control mode: client configuration; `fault_ack`, `ts`, `op_idx_aux`, `aux` must be zero.
- Effort: `cur_ref` for current mode, `tor_ref` otherwise; unused effort field must be zero.
- Position/velocity modes: `gain_2` and `gain_3` must be zero.

## Dependencies

- `rclcpp`
- `sensor_msgs`
- `diagnostic_msgs`
- `std_msgs`
- `std_srvs`
- `advrf_ros2_interfaces`

## TroubleShooting

| Error / symptom | Check / fix |
| --- | --- |
| `Set EC_CFG to the client YAML file` | Export `EC_CFG` before run/launch |
| CMake cannot find EtherCAT packages | Build/install master and client; set `ECAT_INSTALL_PREFIX` |
| `EtherCAT system startup failed` | Master/backend, client network config, device configuration |
| `Missing motor references or too many motors` | Configured motor references; supported count: 1–128 |
| No motor commands transmitted | Enable `set_active_tx`; check namespace and command QoS |
| `Rejected invalid motor command batch` | IDs, duplicates, finite values, effort fields, reserved fields, gains |
| `Motor command watchdog timeout` | Command interval below `command_timeout_ms`; restart bridge after fault |
| `EtherCAT pipe stopped` / `EtherCAT writer stopped` | Backend/client connection; restart bridge after fault |
