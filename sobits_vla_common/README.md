# sobits_vla_common

## Purpose

The shared hub for the VLA tools pipeline: robot descriptors, the ROS
parameter-schema loader, the LeRobot version-compat seam, gamepad input, and
world-reset. Every other package (`sobits_vla_rosbag_collection`,
`sobits_vla_rosbag_conversion`, `sobits_vla_training`, `sobits_vla_deploy`,
`sobits_vla_visualization`) depends on this one; it depends on none of them.
It sits "below" the rosbag → dataset → training → deploy → eval pipeline,
supplying the config/robot-descriptor plumbing every stage reads.

## Nodes / executables

| Executable | Language | Role |
|---|---|---|
| `gamepad_clt_node` | C++ | Translates `/joy` button presses into `sobits_interfaces/srv/VlaCommand` requests against the collection or deploy stage. |
| `world_reset_node` | Python (`scripts/world_reset_node`) | Teleports the scene/robot back to a preset between episodes in Gazebo. |
| `new_robot` | Python (`scripts/new_robot`) | Scaffolds a new robot descriptor YAML from a template. |

None of these are meant to be run standalone in production — they are
brought up by the collection/deploy launch files (see those packages'
READMEs) or invoked directly for scaffolding.

## Parameters

`sobits_vla_common/param_schema.py` is the schema loader every other
package's node uses (`declare_from_schema(node, SCHEMA)` /
`read_schema(node, SCHEMA)` / `validate_config(SCHEMA, yaml_path)`) — see
`sobits_vla_common/sobits_vla_common/param_schema.py`. This package's own
nodes:

- `gamepad_clt.cpp` — hand-declared (C++, no schema loader yet):
  `gamepad.command_service`, `gamepad.controller`,
  `gamepad.button_cooldown_duration`, `gamepad.deploy_service_match`, plus a
  per-controller `gamepad.<controller>.button_mapping.*` tree.
- `world_reset_node.py` — scene/model names, reset presets, and the
  `robot.*` tree (joint reset poses) read from
  `sobits_vla_common/config/world_reset_<robot>.yaml`.

## Topics / services

Owner-private naming: a node advertises `~/<channel>`, which resolves to
`/<node_name>/<channel>` bare or `/<robot>/<node_name>/<channel>`
namespaced. Consumers address it by the owner's relative name,
`<owner_node>/<channel>`.

| Node | Advertises | Consumes |
|---|---|---|
| `gamepad_clt_node` | — | `joy` (`sensor_msgs/Joy`, robot I/O, absolute via remap); `gamepad.command_service` param, e.g. `sobits_vla_deploy/command` or `vla_rosbag_collection/command` |
| `world_reset_node` | `~/reset_world` (`sobits_interfaces/srv/VlaResetWorld`) | robot model/joint topics from the descriptor (absolute) |

## Outputs

This package writes no dataset/model/log artifacts of its own — see the
per-package Outputs sections for the `output_root()` convention
(`sobits_vla_common/sobits_vla_common/output_root.py`) that conversion, training, and deploy's
`eval/` use to resolve their own output roots.

## How to run

```
ros2 run sobits_vla_common gamepad_clt_node --ros-args -p gamepad.command_service:=vla_rosbag_collection/command
ros2 run sobits_vla_common new_robot --help
```

In practice these are brought up by `rosbag_collection.launch.py` /
`sobits_vla_deploy.launch.py` in their own packages, not run standalone.

## How to test

```
# Inside the jazzy_sobit_home_2_moveit_ws container:
cd /home/rg-station-04-keith/colcon_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select sobits_vla_common --test-result-base build/sobits_vla_common
colcon test-result --test-result-base build/sobits_vla_common

# Pure-Python unit tests directly via pixi (no colcon rebuild needed):
cd src/sobits_vla_tools
pixi run -e gpu python -m pytest sobits_vla_common/test/ -q
```
