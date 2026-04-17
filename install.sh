#!/bin/bash

set -e

echo "╔══╣ Install: SOBITS VLA TOOLS (STARTING) ╠══╗"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

if [ -x /opt/pytorch/.venv/bin/python3 ]; then
    PYTHON_BIN=/opt/pytorch/.venv/bin/python3
else
    PYTHON_BIN=$(command -v python3)
fi

# Version specs can be overridden per environment, e.g.:
#   LEROBOT_VERSION_SPEC='~=0.5.1' NUMPY_VERSION_SPEC='>=2.0.0,<2.3.0' ./install.sh
LEROBOT_VERSION_SPEC=${LEROBOT_VERSION_SPEC:-"~=0.5.1"}
# lerobot==0.5.1 requires numpy>=2.0.0,<2.3.0.
NUMPY_VERSION_SPEC=${NUMPY_VERSION_SPEC:-">=2.0.0,<2.3.0"}
NUMEXPR_VERSION_SPEC=${NUMEXPR_VERSION_SPEC:-">=2.10.2"}
BOTTLENECK_VERSION_SPEC=${BOTTLENECK_VERSION_SPEC:-">=1.4.2"}
TORCH_VERSION_SPEC=${TORCH_VERSION_SPEC:-""}

PIP_ARGS=()
# Ubuntu 24+ can mark system Python as externally managed (PEP 668).
# Only add the override when not using a virtual environment.
if ! "${PYTHON_BIN}" -c 'import sys; raise SystemExit(0 if sys.prefix != sys.base_prefix else 1)'; then
    PIP_ARGS+=(--break-system-packages)
fi

cd "${WORKSPACE_ROOT}"

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS_DISTRO is not set. Source your ROS 2 environment first."
    exit 1
fi

sudo apt update -y
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-trajectory-msgs

rosdep update
rosdep install --from-paths sobits_vla_tools --ignore-src -r -y

PYTHON_PACKAGES=(
    "huggingface_hub"
    "lerobot${LEROBOT_VERSION_SPEC}"
    "numpy${NUMPY_VERSION_SPEC}"
    "numexpr${NUMEXPR_VERSION_SPEC}"
    "bottleneck${BOTTLENECK_VERSION_SPEC}"
    "pyyaml"
    "rosbags"
    "scipy"
)

echo "Using Python: ${PYTHON_BIN}"
echo "LeRobot spec: ${LEROBOT_VERSION_SPEC}"
echo "NumPy spec: ${NUMPY_VERSION_SPEC}"

"${PYTHON_BIN}" -m pip install "${PIP_ARGS[@]}" -U pip
"${PYTHON_BIN}" -m pip install "${PIP_ARGS[@]}" "${PYTHON_PACKAGES[@]}"

if ! "${PYTHON_BIN}" -c "import torch" >/dev/null 2>&1; then
    TORCH_PACKAGE="torch"
    if [ -n "${TORCH_VERSION_SPEC}" ]; then
        TORCH_PACKAGE="torch${TORCH_VERSION_SPEC}"
    fi
    "${PYTHON_BIN}" -m pip install "${PIP_ARGS[@]}" "${TORCH_PACKAGE}"
fi

echo "╚══╣ Install: SOBITS VLA TOOLS (FINISHED) ╠══╝"