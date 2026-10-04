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
| `model.*` | `repo_id`, `policy_class`, `device`, `use_amp`, `use_relative_actions`, `fake_policy` (hold-pose stand-in, no lerobot/torch; see "Dry run"), `action_space` (`joint` or `ee`), `ee_rotation` (`rotvec` or `rpy`, must match the checkpoint's `ee.*` names). |
| `ee_servo.*` | `max_lin_step_m`, `max_ang_step_rad` -- per-step clamp on EE servo targets in `action_space: ee` mode. |
| `runtime.*` | `control_hz`, `actions_per_chunk`, `async_enabled`, `action_interpolation_multiplier`. |
| `rtc.*` | Real-Time Chunking guidance knobs. |
| `status.*` | `rate_hz` (default `1.0`) -- heartbeat rate of `~/status`. |
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
| `~/status` (`sobits_interfaces/VlaStatus`, reliable + transient-local, depth 1) | — |

`vla_experiment_runner` is a consumer: `command_service` defaults to
`sobits_vla_deploy/command`, `episode_done_topic` to
`sobits_vla_deploy/episode_done`.

## Status feed

`~/status` publishes `VlaStatus` with `stage = STAGE_DEPLOY`: a heartbeat at
`status.rate_hz` (steady clock, so a paused sim does not stall it) plus one
message immediately on every event. Fields: `state` (`STOPPED`, `PLAYING`,
`RESETTING`, `ERROR`), `event` and `event_seq`, `task_name`, `episode_name`
(`episode_<n>` of this process), `elapsed_sec` (the node's episode clock; with
a deadman it starts at the first engagement and freezes at stop), `policy`,
`deadman_enabled` / `deadman_engaged`, `steps`, `inference_hz` (rolling window
of chunk inferences, 0 when idle) and `outcome`.

| Trigger | State | Event |
|---|---|---|
| PLAY | `PLAYING` | `STARTED` |
| deadman pressed / released | `PLAYING` | `ENGAGED` / `RELEASED` |
| STOP while playing, or auto-termination | `RESETTING` | `STOPPED` (outcome set) |
| world reset finished after an episode | `STOPPED` | `EPISODE_DONE` (outcome set; mirrors `~/episode_done`) |
| idle STOP / RESET | `RESETTING`, then `STOPPED` | none, then `RESET_DONE` |
| task set (`~/task`, `~/update_task`) | unchanged | `TASK_SET` |
| unsupported command | unchanged | `REJECTED` |

The deadman latch is cleared on every PLAY and STOP, so a trigger still held
across STOP -> PLAY engages again instead of leaving the robot frozen.

## Dry run (`model.fake_policy`)

`model.fake_policy: true` swaps the checkpoint for a stand-in that holds the
measured pose (zero for `*.vel` keys), so PLAY/STOP/RESET, the deadman and the
status feed can run in the sim without lerobot, torch or pixi:

```
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py deploy_config:=deploy_config_sobit_home_fake fake_policy:=true use_sim_time:=true robot_name:=sobit_home
```

The `fake_policy` launch arg runs the node on the system python
(`PYTHONNOUSERSITE=1`, no pixi prefix). `deploy_config_sobit_home_fake.yaml`
excludes the right arm and the hand cameras, so only `head_camera` must stream.

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
(`deploy_config`/`config_file`, `controller`, `model_*`, `enable_servo_backend`
for the first; `num_episodes`, `episode_timeout_s`, `done_wait_margin_s` for
the second).

### EE action mode (`model.action_space: ee`)

`enable_servo_backend:=true` brings up `sobits_teleop`'s
`arm_backend_servo.launch.py` (MoveIt Servo + `servo_target_bridge`) so the
deploy node can stream `ee.{arm}.*` actions as TF targets instead of joint
commands. `/<robot_name>/move_group` must already be running — the include
fetches robot description/kinematics params from it and waits up to 60 s
before aborting. If the deploy node dies or stops publishing, the bridge
holds the last commanded TF target until servo's `incoming_command_timeout`
(0.5 s) pauses motion — it does not freeze instantly. The bridge also clamps
commanded targets to a 1.10 m reach from its configured origin frame.

`model.ee_rotation` (`rotvec`, default, or `rpy`) must match the checkpoint's
`ee.*` feature names; the loader refuses a mismatch, and `quat` checkpoints
are not deployable (ObsBuilder and the servo publisher handle rotvec and rpy
only).

#### Relative EE actions

Datasets store absolute EE poses. A checkpoint trained with
`robot.ee_relative_actions: true` (sobits_vla_training) carries the sobits
`sobits_ee_relative_actions` / `sobits_ee_absolute_actions` processor pair,
so each predicted step is a pose relative to the observation pose
(UMI-style, `A_k = inv(T_obs) · T_k`) and `postprocessor()` composes it back
as `T_obs · A_k` on the observation the preprocessor cached. Nothing needs
to be set in the deploy config; the loader re-pairs the steps and refuses:

- LeRobot's per-component `use_relative_actions` on `ee.*` names that are
  not in `relative_exclude_joints` (a rotation cannot be subtracted per
  component);
- an EE relative step without a preprocessor, or an unpaired step;
- `rtc.enabled` with any relative model — the RTC prefix from the previous
  chunk is anchored on the previous observation and is not re-anchored yet.

### Limitations

Composed targets are absolute in the descriptor's EE `reference_frame`
(`body_lift_link` on sobit_home); if the base drives during a chunk they go
stale (same as absolute mode). `ee_servo.max_lag_m` / `max_lag_rad` bound how
far a target may lead the measured pose, so a stalled arm does not bank a
chunk's worth of steps; non-finite targets are dropped. RTC is unavailable with
relative models until the prefix is re-anchored.

## How to test

```
cd /home/rg-station-04-keith/colcon_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select sobits_vla_deploy --test-result-base build/sobits_vla_deploy
colcon test-result --test-result-base build/sobits_vla_deploy

cd src/sobits_vla_tools
pixi run -e gpu python -m pytest sobits_vla_deploy/test/ -q --ignore-glob='*test_flake8.py' --ignore-glob='*test_pep257.py' --ignore-glob='*test_copyright.py'
```

Without torch/pandas (the colcon container) `conftest.py` skips the modules
that need them; the pixi run covers those. `test_deploy_status.py`,
`test_deadman.py` and `test_fake_policy.py` are pure; `test_deploy_status_wire.py`
checks the `VlaStatus` constants and `to_msg` mapping against the built message.

`test_vla_deploy_unit.py` covers `ActionChunkBuffer`, `ActionInterpolator`,
episode-logger termination logic; `test_eval_metrics.py` /
`test_eval_resample_context.py` cover `eval/metrics.py` against synthetic
episode logs with hand-computed expected values.
