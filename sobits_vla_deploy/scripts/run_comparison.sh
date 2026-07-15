#!/usr/bin/env bash
# Copyright (c) 2026, Team SOBITS — BSD-3-Clause.
#
# Run the full SmolVLA vs Pi05 comparison with a single command.
#
# Assumes Gazebo + the robot are ALREADY running. Each model is launched in a
# fresh deploy process (no hot-swap): SmolVLA first, then Pi05, then the
# analyzer produces IEEE-style tables + figures.
#
# Usage:
#   ./run_comparison.sh [NUM_EPISODES] [LOG_DIR] [OUT_DIR]
# Defaults:
#   NUM_EPISODES=20  LOG_DIR=/tmp/vla_logs  OUT_DIR=/tmp/vla_eval
#
# Override the model configs / labels with env vars if needed:
#   SMOLVLA_CONFIG, SMOLVLA_LABEL, PI05_CONFIG, PI05_LABEL
set -euo pipefail

NUM_EPISODES="${1:-20}"
LOG_DIR="${2:-/tmp/vla_logs}"
OUT_DIR="${3:-/tmp/vla_eval}"

SMOLVLA_CONFIG="${SMOLVLA_CONFIG:-deploy_config_sobit_home_left_smolvla}"
SMOLVLA_LABEL="${SMOLVLA_LABEL:-smolvla}"
PI05_CONFIG="${PI05_CONFIG:-deploy_config_sobit_home_left_pi05}"
PI05_LABEL="${PI05_LABEL:-pi05}"

run_model() {
  local cfg="$1" label="$2"
  echo "=================================================================="
  echo " Running ${label}  (config=${cfg}, episodes=${NUM_EPISODES})"
  echo "=================================================================="
  ros2 launch sobits_vla_deploy vla_experiment.launch.py \
    deploy_config:="${cfg}" \
    model_label:="${label}" \
    num_episodes:="${NUM_EPISODES}" \
    log_dir:="${LOG_DIR}" \
    use_sim_time:=true
}

run_model "${SMOLVLA_CONFIG}" "${SMOLVLA_LABEL}"
run_model "${PI05_CONFIG}" "${PI05_LABEL}"

echo "=================================================================="
echo " Generating comparison report -> ${OUT_DIR}"
echo "=================================================================="
ros2 run sobits_vla_visualization vla_eval \
  --logs "${SMOLVLA_LABEL}:${LOG_DIR}/${SMOLVLA_LABEL}" \
         "${PI05_LABEL}:${LOG_DIR}/${PI05_LABEL}" \
  --out "${OUT_DIR}"

echo "Done. See ${OUT_DIR}/ for tables (csv/md/tex) and figures (png/pdf)."
