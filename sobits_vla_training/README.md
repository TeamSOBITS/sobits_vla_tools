# sobits_vla_training

## Purpose

Wraps LeRobot's training loop as a ROS 2 node: the third pipeline stage
(rosbag → dataset → **training** → deploy → eval). Reads a
`LeRobotDataset` produced by `sobits_vla_rosbag_conversion`, builds a
LeRobot `TrainPipelineConfig` from ROS parameters, and drives
`lerobot.scripts.train` (single-GPU or DDP via `num_gpus`).

## Nodes / executables

| Executable | Role |
|---|---|
| `train_node` | `sobits_vla_training.train_node:main`, node name `sobits_vla_training`. One-shot: builds the config, runs training, exits. |

## Parameters

Schema-driven (`_SCHEMA` in `train_node.py`, ~70 keys across 10 groups — too
many to list here; see
`sobits_vla_training/sobits_vla_training/train_node.py:66`):

| Group | Covers |
|---|---|
| `policy` | Policy type + policy-specific hyperparameters. |
| `dataset` | `repo_id`, root override. |
| `checkpoint` | `output_dir`, `pretrained_path`, `resume`. |
| `training` | `steps`, `batch_size`, optimizer/scheduler knobs. |
| `num_gpus`, `wandb.*`, `hub.*` | DDP fan-out, W&B logging, HF Hub push. |
| `policy_overrides` | Free-form dict merged into the policy config last — the escape hatch for anything not otherwise exposed. |
| `peft.*` | LoRA fine-tuning knobs (rank, alpha, target modules). |
| `robot.*` | Robot descriptor passthrough for policies that need morphology at train time. |

## Topics / services

None advertised — training reads a dataset from disk and writes checkpoints,
it does not participate in the live VLA bus.

## Outputs

Checkpoints land under `<pkg_src>/lerobotmodel/<output_dir>/` by default,
resolved via `output_root('sobits_vla_training', 'lerobotmodel', ...,
final_fallback=find_package_src_dir()/'lerobotmodel')` in `config_builder.py`;
absolute `output_dir` values are used verbatim. `lerobotmodel/` carries the
uniform `*\n!.gitignore\n` ignore.

## How to run

```
ros2 launch sobits_vla_training sobits_vla_training.launch.py robot:=sobit_home_left_smolvla_fft steps:=30000
```

Verified args (`--show-args`): `robot`, `config_file`, `node_name`,
`enable_gpu` (default `true`), `pixi_env`, `pixi_manifest`, plus per-field
overrides (`policy`, `dataset_repo_id`, `output_dir`, `pretrained_path`,
`resume`, `steps`, `batch_size`, `num_gpus`, `wandb_enable`, `wandb_project`,
`hub_repo_id`).

## How to test

```
cd /home/rg-station-04-keith/colcon_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select sobits_vla_training --test-result-base build/sobits_vla_training
colcon test-result --test-result-base build/sobits_vla_training

cd src/sobits_vla_tools
pixi run -e gpu python -m pytest sobits_vla_training/test/ -q --ignore-glob='*test_flake8.py' --ignore-glob='*test_pep257.py' --ignore-glob='*test_copyright.py'
```

Lint-only today (`test_flake8.py`, `test_pep257.py`, `test_copyright.py`) —
no unit tests of `train_node.py`'s logic yet.
