# sobits_vla_rosbag_collection

## Purpose

C++ node that records teleoperated demonstrations into rosbags: the first
stage of the pipeline (rosbag → dataset → training → deploy → eval). It
subscribes to the robot's own topics (joint states, cameras, cmd_vel — all
absolute, from the robot descriptor), records them via `rosbag2_transport`,
and writes `recorded_bags_meta.yaml` alongside each episode so
`sobits_vla_rosbag_conversion` can find and validate them later.

## Nodes / executables

| Executable | Class | Role |
|---|---|---|
| `rosbag_collection_node` | `sobits_vla::RosbagCollection` | Owns the recorder, task/episode bookkeeping, and the `VlaCommand` service that drives record/pause/save/delete/reset. |

Internally split into `episode_lifecycle.{hpp,cpp}` (bag naming, min/max
duration + integrity checks, ROS-free), `rosbag_collection_params.cpp` (the
~40 `declare_parameter` calls), `bag_metadata_manager.cpp`,
`recording_monitor.cpp`, `topic_builder.cpp`, and `robot_descriptor_loader.cpp`.

## Parameters

Hand-declared C++ params (no schema loader yet in this package), grouped:

| Group | Examples |
|---|---|
| `robot_descriptor_id` | Selects the shared descriptor under `sobits_vla_common/robots/`; falls back to a legacy inline `robot_info.*` tree if empty. |
| `rosbag_config.*` | `record_directory`, `min_episode_duration`, `max_episode_duration`, `expected_sensor_fps`, `min_disk_space_mb`, `additional_topics`, `conversion_format`, `world_reset_service`. |
| `user_info.*` | `name`, `email`, `location` — written into `recorded_bags_meta.yaml`. |
| `gamepad.*` | `command_service` (the service this node itself advertises, `~/command`), `controller`. |

See `sobits_vla_rosbag_collection/src/rosbag_collection_params.cpp` for the
full list.

## Topics / services

Owner-private naming (see `sobits_vla_common`'s README): this node's name is
`vla_rosbag_collection`.

| Advertises | Consumes |
|---|---|
| `~/command` (`VlaCommand`) — resolves to `/<robot>/vla_rosbag_collection/command` | Robot I/O (joint states, cameras, cmd_vel — absolute, from the descriptor) |
| `~/vla_task_update`, `~/vla_subtask_update` (`VlaUpdateTask`) | `world_reset_node/reset_world` (consumer form; only when `enable_world_reset:=true`) |

## Outputs

Rosbags land under `<pkg_src>/rosbags/<task>/<episode>/` by default
(`record_directory:=` overrides). `sobits_vla_rosbag_collection/rosbags/`
carries the uniform `*\n!.gitignore\n` ignore — everything under it except
`.gitignore` itself is gitignored. `scripts/play_all_bags.sh` replays a
recorded set for RViz2 inspection.

## How to run

```
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py enable_world_reset:=false robot_name:=sobit_home
```

Verified args (`ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py --show-args`):
`record_directory`, `robot_name` (default `sobit_home`), `use_sim_time`,
`enable_world_reset` (default `false`).

## How to test

```
cd /home/rg-station-04-keith/colcon_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --packages-select sobits_vla_rosbag_collection --allow-overriding sobits_vla_rosbag_collection
colcon test --packages-select sobits_vla_rosbag_collection --test-result-base build/sobits_vla_rosbag_collection
colcon test-result --test-result-base build/sobits_vla_rosbag_collection
```

gtest suites: `test_episode_lifecycle.cpp`, `test_topic_builder.cpp`,
`test_bag_metadata_manager.cpp` (`sobits_vla_rosbag_collection/test/`).
