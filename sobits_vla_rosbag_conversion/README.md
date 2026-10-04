# sobits_vla_rosbag_conversion

## Purpose

Converts recorded rosbags into a LeRobotDataset: the second stage of the
pipeline (rosbag → **dataset** → training → deploy → eval). Reads the
episodes + `recorded_bags_meta.yaml` written by `sobits_vla_rosbag_collection`,
resamples every sensor/actuator stream onto a common time grid per the robot
descriptor, and writes a HuggingFace `LeRobotDataset` that
`sobits_vla_training` trains against.

## Nodes / executables

| Executable | Role |
|---|---|
| `ros2bag_to_lerobotdataset` | Console-script name (stable); runs `sobits_vla_rosbag_conversion.conversion_node:main`, node name `rosbag_conversion_node`. One-shot: converts on startup, then exits. |

`conversion_node.py` is the thin ROS wrapper; the actual conversion logic
lives in `pipeline/` (`discovery.py`, `validator.py`, `episode_pipeline.py`,
`stats.py`) and `sync/` (`core.py`, `images.py`, `joints.py`, `poses.py`),
composed by the `FrameSynthesizer` facade in `frame_synthesizer.py`. None of
`pipeline/`/`sync/` import `rclpy`.

## Parameters

Schema-driven (`_SCHEMA` in `conversion_node.py`, no dynamic sections):
`rosbag_directory`, `recorded_bags_meta_file`, `dataset_name`,
`output_directory`, `fps`, `vcodec`, `sync_threshold`,
`downsample_tolerance`, `push_to_hub`, `skip_static_threshold`,
`exclude.{groups,cameras,ee}` (trims the shared descriptor),
`cameras.primary`, `robot_descriptor_id`, `ee_actions.{arms,rotation}`. See
`sobits_vla_rosbag_conversion/sobits_vla_rosbag_conversion/conversion_node.py:77`.

Actions are always **absolute**. `ee_actions.rotation` picks the EE rotation
encoding: `rotvec` (default, `ee.<arm>.{x,y,z,rx,ry,rz}`), `rpy` (scipy `xyz`
extrinsic, `...roll,pitch,yaw`) or `quat` (`...qx,qy,qz,qw`); poses are in each
ee entry's `target_frame`. `use_relative_actions: true` and `ee_actions.frame`
were removed and now raise: relative actions are a training-time option
(`robot.ee_relative_actions`, `policy_overrides.use_relative_actions`). The
convention is recorded as `action_convention` in `meta/sobits_vla_info.json`
and `conversion_stats.yaml`.

## Topics / services

None advertised — this node reads rosbags from disk and writes a dataset,
it does not participate in the live VLA bus.

## Outputs

Datasets land under `<pkg_src>/lerobotdataset/<dataset_name>/` by default,
resolved via `output_root('sobits_vla_rosbag_conversion', 'lerobotdataset')`
(`sobits_vla_common/sobits_vla_common/output_root.py`); `output_directory:=` overrides.
`lerobotdataset/` carries the uniform `*\n!.gitignore\n` ignore.

## How to run

```
ros2 launch sobits_vla_rosbag_conversion rosbag_conversion.launch.py robot:=sobit_home
```

`ros2 launch` relays the node's output one complete line at a time, so
progress is printed as a line every 10 s (`episodes: 40%|... 80/200`), never
as an in-place bar. For live bars run the node directly in a terminal:

```
P=$(ros2 pkg prefix sobits_vla_rosbag_conversion)
pixi run -e gpu python $P/lib/sobits_vla_rosbag_conversion/ros2bag_to_lerobotdataset \
  --ros-args --params-file $P/share/sobits_vla_rosbag_conversion/config/conversion_config_sobit_home_left_ee.yaml \
  -p overwrite:=true
```

Verified args (`--show-args`): `robot`, `config_file`, `rosbag_directory`,
`recorded_bags_meta_file`, `dataset_name`, `vcodec`, `overwrite`,
`enable_gpu` (default `true`), `pixi_env`, `pixi_manifest`. `enable_gpu:=true`
runs the node under the `gpu` pixi env (needed for the torch/lerobot
dependencies used when writing dataset stats).

## How to test

```
cd /home/rg-station-04-keith/colcon_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select sobits_vla_rosbag_conversion --test-result-base build/sobits_vla_rosbag_conversion
colcon test-result --test-result-base build/sobits_vla_rosbag_conversion

cd src/sobits_vla_tools
pixi run -e gpu python -m pytest sobits_vla_rosbag_conversion/test/ -q --ignore-glob='*test_flake8.py' --ignore-glob='*test_pep257.py' --ignore-glob='*test_copyright.py'
```

`test_conversion_golden.py` is the regression net: a deterministic synthetic
mcap fixture (`make_fixture_bag.py`) converted and compared byte-for-byte
(features dict, stats, parquet, ffprobe) across runs. `test_offline_tf_tree.py`,
`test_episode_discovery.py`, `test_conversion_validator.py`,
`test_conversion_stats.py` cover the `pipeline/`/`sync/` modules directly.
