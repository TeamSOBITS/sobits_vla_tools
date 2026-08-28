# sobits_vla_deploy

## Purpose

Runs a trained VLA policy on the robot: the fourth and fifth pipeline
stages (rosbag → dataset → training → **deploy** → **eval**). The deploy
node loads a LeRobot checkpoint, builds observations from live robot
topics, runs inference, and executes actions on the real/sim robot; the
`eval/` subpackage separately analyzes the episode logs it writes.

## Nodes / executables

| Executable | Class / module | Role |
|---|---|---|
| `sobits_vla_deploy` | `deploy_node.py`, class `LeRobotDeployNode` (node name `sobits_vla_deploy`) | Loads the policy, runs the control loop, exposes PLAY/STOP/RESET. |
| `vla_experiment_runner` | `experiment_runner.py` (node name `vla_experiment_runner`) | Drives N episodes unattended against a running deploy node. |
| `vla_eval` | `eval/cli.py:main` | Offline analysis of episode logs — tables + figures, no ROS. |

`deploy_node.py` is a thin wrapper; runtime collaborators are
`policy_loader.py`, `obs_builder.py`, `inference_engine.py`,
`action_chunk_buffer.py`, `action_interpolator.py`, `action_executor.py`, and
`episode_logger.py` (facade over the `episode_logging/` subpackage:
`scene_probe.py`, `step_log.py`, `termination.py`). `sobits_vla_deploy.py`
and `vla_episode_logger.py` are deprecated import shims — new code should
import `deploy_node` / `episode_logger` directly.

## Parameters

Schema-driven (`_SCHEMA` in `deploy_node.py`; see
`sobits_vla_deploy/sobits_vla_deploy/deploy_node.py:69`):

| Group | Covers |
|---|---|
| `model.*` | `repo_id`, `policy_class`, `device`, `use_amp`, `use_relative_actions`. |
| `runtime.*` | `control_hz`, `actions_per_chunk`, `async_enabled`, `action_interpolation_multiplier`. |
| `rtc.*` | Real-Time Chunking guidance knobs. |
| `gamepad.*` | `command_service` (this node advertises `~/command`), `controller`, per-controller `button_mapping`, deadman safety trigger. |
| `logging.*` | Episode-log directory + scene-config YAML for termination baselines. |
| `task.*` | Pick/place success thresholds, timeout, tilt/fall detection. |
| `reset.*` | `world_service` (consumer form: `world_reset_node/reset_world`). |

`robot.*` (joints/topics/cameras) comes from the shared robot descriptor,
not this schema — see `sobits_vla_common`.

## Topics / services

Owner-private naming: node name `sobits_vla_deploy`.

| Advertises | Consumes |
|---|---|
| `~/play` (Bool), `~/task` (String) | Robot I/O (joint states, cameras, cmd_vel — absolute) |
| `~/update_task` (`VlaUpdateTask`), `~/command` (`VlaCommand`) | `world_reset_node/reset_world` (consumer form) |
| `~/episode_done` (String) | — |

`vla_experiment_runner` is a consumer: `command_service` defaults to
`sobits_vla_deploy/command`, `episode_done_topic` to
`sobits_vla_deploy/episode_done`.

## Outputs

Episode logs land under `<pkg_src>/logs/<model_label>/` by default,
resolved via `output_root('sobits_vla_deploy', 'logs')`; `eval/cli.py`'s
`--out` defaults to that same logs root's `eval/` subdirectory when omitted.
`logs/` carries the uniform `*\n!.gitignore\n` ignore.

## How to run

```
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py deploy_config:=deploy_config_sobit_home robot_name:=sobit_home
ros2 launch sobits_vla_deploy vla_experiment.launch.py deploy_config:=deploy_config_sobit_home_left_smolvla num_episodes:=20
```

Verified args (`--show-args`, both files): `enable_gpu` (default `true`,
runs under the `gpu` pixi env — needed for torch/lerobot), `pixi_env`,
`pixi_manifest`, `robot_name`, `enable_world_reset`, plus per-file overrides
(`deploy_config`/`config_file`, `controller`, `model_*` for the first;
`num_episodes`, `episode_timeout_s`, `done_wait_margin_s` for the second).

## How to test

```
cd /home/rg-station-04-keith/colcon_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select sobits_vla_deploy --test-result-base build/sobits_vla_deploy
colcon test-result --test-result-base build/sobits_vla_deploy

cd src/sobits_vla_tools
pixi run -e gpu python -m pytest sobits_vla_deploy/test/ -q --ignore-glob='*test_flake8.py' --ignore-glob='*test_pep257.py' --ignore-glob='*test_copyright.py'
```

`test_vla_deploy_unit.py` covers `ActionChunkBuffer`, `ActionInterpolator`,
episode-logger termination logic; `test_eval_metrics.py` /
`test_eval_resample_context.py` cover `eval/metrics.py` against synthetic
episode logs with hand-computed expected values.
