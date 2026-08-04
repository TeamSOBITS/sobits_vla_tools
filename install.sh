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
#     script installs pixi if missing and materializes the environments from
#     pixi.lock. No global pip, no --break-system-packages.
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
# 3. Materialize the pixi environments from pixi.toml / pixi.lock.
#    Install every environment so both CPU and GPU machines are covered; on a
#    CPU-only host you may instead install just the -cpu envs, e.g.:
#      pixi install -e viz -e training-cpu -e deploy-cpu -e conversion-cpu
# -----------------------------------------------------------------------------
cd "${SCRIPT_DIR}"
pixi install --all
cd "${DIR}"

echo "╚══╣ Install: SOBITS VLA TOOLS (FINISHED) ╠══╝"
echo
echo "Next:"
echo "  1. Source ROS:   source /opt/ros/\${ROS_DISTRO}/setup.bash"
echo "  2. rosdep:       rosdep install --from-paths . --ignore-src -r -y"
echo "  3. Build:        colcon build"
echo "  4. Run a node:   pixi run -e training-gpu ros2 run sobits_vla_training train_node"
