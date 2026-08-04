#!/bin/bash
# Sourced by pixi [activation]. Bridges the apt-installed ROS 2 (Jazzy) into
# the active pixi environment so rclpy and message bindings import.
#
# ROS itself is NOT managed by pixi — it comes from `apt install ros-jazzy-*`
# and rosdep. This only makes it visible from inside the pixi env.
#
# Requires ROS_DISTRO to be set (export it or source /opt/ros/setup.sh before
# `pixi run`/`pixi shell`).

# Disable the per-user site (~/.local/lib/pythonX/site-packages). The container
# has an apt/pip user-site numpy 1.26 there; Python puts ~/.local AHEAD of the
# pixi env site, so without this the pixi numpy 2.x is shadowed and lerobot/
# pandas crash with "numpy.dtype size changed" ABI errors. This is the single
# most important line in the bridge.
export PYTHONNOUSERSITE=1

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "[pixi_ros_bridge] ROS_DISTRO not set — source /opt/ros/<distro>/setup.sh first." >&2
    return 0 2>/dev/null || exit 0
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${ROS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
else
    echo "[pixi_ros_bridge] ${ROS_SETUP} not found." >&2
fi

# Sourcing ROS prepends /opt/ros + gz-vendor + system lib dirs to
# LD_LIBRARY_PATH, which then shadow the pixi env's own shared libs. That makes
# the env's Python _ssl.so load the SYSTEM libcrypto.so.3 (OpenSSL 3.0.x)
# instead of the env's (3.6.x) -> "OPENSSL_3.3.0 not found" when pyarrow/ssl
# load. Re-prepend the env's lib dir so the env's libs win again. CONDA_PREFIX
# is set by pixi to the active environment root.
if [ -n "${CONDA_PREFIX:-}" ] && [ -d "${CONDA_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

# Ensure ROS python (rclpy, message packages) is importable. The pixi
# interpreter is 3.12 to match Jazzy's ABI.
ROS_SITE="/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages"
if [ -d "${ROS_SITE}" ]; then
    export PYTHONPATH="${ROS_SITE}${PYTHONPATH:+:${PYTHONPATH}}"
fi

# Drop the container's system OpenCV from PYTHONPATH (set globally by
# /etc/profile.d/opencv.sh -> /tmp/opencv_pkg_install). Its cv2 .so links
# against the SYSTEM glib, but a PYTHONPATH entry is imported ahead of the pixi
# env site, so it shadows the env's opencv-python-headless and then dlopens the
# pixi env's older libglib -> "undefined symbol: g_variant_builder_init_static".
# The pixi env already provides opencv-python-headless; let that satisfy cv2.
if [ -n "${PYTHONPATH:-}" ]; then
    _clean_pp=""
    IFS=':' read -ra _pp_entries <<< "${PYTHONPATH}"
    for _e in "${_pp_entries[@]}"; do
        case "${_e}" in
            */opencv_pkg_install/*|"") : ;;   # skip system opencv and empties
            *) _clean_pp="${_clean_pp:+${_clean_pp}:}${_e}" ;;
        esac
    done
    export PYTHONPATH="${_clean_pp}"
    unset _clean_pp _pp_entries _e
fi
