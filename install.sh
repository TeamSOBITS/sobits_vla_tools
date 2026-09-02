#!/bin/bash

set -e

echo "╔══╣ Install: SOBITS VLA TOOLS (STARTING) ╠══╗"

# =============================================================================
# SCOPE: non-ROS setup only.
#
#   * ROS packages (ros-jazzy-desktop, message pkgs, rclpy, cv_bridge, tf2_ros,
#     launch, ament_*, python3-opencv ...) are NOT installed here. They come
#     from apt (`apt install ros-jazzy-desktop`) + rosdep, driven by each
#     package.xml / CMakeLists.txt. This script does not run apt for ROS and
#     does not run rosdep.
#
#   * Python (non-ROS) dependencies are managed by pixi (see pixi.toml). This
#     script installs pixi if missing and materializes ONE environment from
#     pixi.lock: `gpu` if an NVIDIA GPU with a working driver is visible,
#     otherwise `cpu`. No global pip, no --break-system-packages.
#     Force a choice with SOBITS_VLA_PIXI_ENV=cpu|gpu bash install.sh
#
#   * Non-ROS source packages this workspace needs (e.g. sobits_interfaces)
#     are cloned so colcon/rosdep can build them.
#
# Run from the sobits_vla_tools package directory.
# =============================================================================

# Keep track of the current directory
DIR=$(pwd)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# -----------------------------------------------------------------------------
# 1. Clone non-ROS source dependencies (built by colcon, resolved by rosdep).
# -----------------------------------------------------------------------------
cd ../

src_packages=(
    "sobits_interfaces"
)

for ((i = 0; i < ${#src_packages[@]}; i++)); do
    if [ -d "${src_packages[i]}" ]; then
        echo "Skipping clone: ${src_packages[i]} already exists."
    else
        echo "Cloning: ${src_packages[i]}"
        git clone --recurse-submodules -b "$ROS_DISTRO-devel" \
            "https://github.com/TeamSOBITS/${src_packages[i]}"
    fi

    if [ -f "${src_packages[i]}/install.sh" ]; then
        echo "Running install.sh in ${src_packages[i]}."
        (cd "${src_packages[i]}" && bash install.sh)
    fi
done

cd "${DIR}"

# -----------------------------------------------------------------------------
# 2. pixi — Python dependency manager (non-ROS).
# -----------------------------------------------------------------------------
if ! command -v pixi >/dev/null 2>&1; then
    echo "pixi not found — installing pixi."
    curl -fsSL https://pixi.sh/install.sh | bash
    # pixi installs to ~/.pixi/bin; make it available for the rest of this run.
    export PATH="${HOME}/.pixi/bin:${PATH}"
fi

echo "Using pixi: $(command -v pixi) ($(pixi --version))"

# -----------------------------------------------------------------------------
# 3. Pick the accelerator environment.
#    Only ONE of `cpu` / `gpu` is installed: they differ solely in the torch
#    wheel index, and each is ~8 GB, so installing both duplicates the whole
#    ML stack. `nvidia-smi -L` is the probe -- it lists devices only when a
#    driver is actually loaded and reachable (true inside a container started
#    with --gpus), so a machine with a card but no usable driver correctly
#    falls back to cpu instead of getting an unusable cu128 env.
# -----------------------------------------------------------------------------
PIXI_ENV="${SOBITS_VLA_PIXI_ENV:-}"

if [ -n "${PIXI_ENV}" ]; then
    case "${PIXI_ENV}" in
        cpu|gpu) echo "Using pixi environment: ${PIXI_ENV} (forced via SOBITS_VLA_PIXI_ENV)." ;;
        *) echo "SOBITS_VLA_PIXI_ENV must be 'cpu' or 'gpu' (got '${PIXI_ENV}')." >&2; exit 1 ;;
    esac
elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L 2>/dev/null | grep -q '^GPU'; then
    PIXI_ENV="gpu"
    echo "NVIDIA GPU detected:"
    nvidia-smi -L 2>/dev/null | sed 's/^/  /'
    echo "Using pixi environment: gpu (CUDA 12.8 torch wheels)."
else
    PIXI_ENV="cpu"
    echo "No usable NVIDIA GPU found (nvidia-smi absent or lists no device)."
    echo "Using pixi environment: cpu (CPU-only torch wheels)."
fi

# -----------------------------------------------------------------------------
# 4. Materialize that environment from pixi.toml / pixi.lock.
# -----------------------------------------------------------------------------
cd "${SCRIPT_DIR}"
pixi install -e "${PIXI_ENV}"
cd "${DIR}"

echo "╚══╣ Install: SOBITS VLA TOOLS (FINISHED) ╠══╝"
echo
echo "Next:"
echo "  1. Source ROS:   source /opt/ros/\${ROS_DISTRO}/setup.bash"
echo "  2. rosdep:       rosdep install --from-paths . --ignore-src -r -y"
echo "  3. Build:        colcon build"
echo "  4. Run a node:   pixi run -e ${PIXI_ENV} ros2 run sobits_vla_training train_node"
