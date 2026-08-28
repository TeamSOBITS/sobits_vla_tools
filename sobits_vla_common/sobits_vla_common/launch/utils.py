# Copyright (c) 2026, Team SOBITS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


"""
Shared launch helpers for sobits_vla_tools.

pixi manages the Python (non-ROS) dependencies of each package in its own
environment (see pixi.toml at the sobits_vla_tools root). A launch file can
therefore start a node in a *specific* pixi env — independent of whichever env
launched `ros2 launch` — by setting the Node `prefix=` to a `pixi run` command.
The spawned ros2 process (and the node) inherits that env's PATH/PYTHONPATH.

This is what lets a bringup launch in package A start package B's node in B's
own env instead of leaking A's env down the process tree.

There are exactly two environments, `cpu` and `gpu` (see pixi.toml) — one shared
stack, differing only in the torch wheel. Launch files expose an ``enable_gpu``
boolean and map it through :func:`pixi_env_for`.
"""

import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from sobits_vla_common.output_root import output_root


def _search_up_for(filename: str, start: str) -> str:
    """Walk up from `start` returning the dir-joined `filename` if found."""
    cur = os.path.abspath(start)
    while True:
        candidate = os.path.join(cur, filename)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(cur)
        if parent == cur:  # reached filesystem root
            return ''
        cur = parent


def default_pixi_manifest() -> str:
    """
    Locate the sobits_vla_tools pixi.toml.

    A source-relative path does NOT survive `colcon build`: after install this
    module lives under install/.../lib/python3.12/site-packages/, nowhere near
    the source-tree pixi.toml. So resolution is, in order:

      1. SOBITS_VLA_PIXI_MANIFEST env var (explicit override — wins).
      2. Search upward for `pixi.toml` from a set of anchors that work in both
         layouts: the current working directory (where `ros2 launch` runs, i.e.
         inside the workspace), each colcon workspace src root derived from
         COLCON_PREFIX_PATH, and finally this file's own directory (covers the
         --symlink-install case where the module still points into src).

    Returns the found path, or the literal 'pixi.toml' as a last resort so the
    error message is legible if nothing matched.
    """
    override = os.environ.get('SOBITS_VLA_PIXI_MANIFEST')
    if override:
        return override

    # 1) --symlink-install: this module's real path is inside the source tree,
    #    so an upward search from here reaches src/sobits_vla_tools/pixi.toml.
    up = _search_up_for('pixi.toml', os.path.dirname(os.path.abspath(__file__)))
    if up:
        return up

    # 2) Non-symlink install: derive <ws> from colcon, then probe the known
    #    source subpath and the CWD subtree (ros2 launch runs from the ws root).
    candidates = []
    for prefix in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        ws = os.path.dirname(prefix.rstrip(os.sep))  # <ws>/install -> <ws>
        candidates.append(os.path.join(ws, 'src', 'sobits_vla_tools', 'pixi.toml'))
    cwd = os.getcwd()
    candidates.append(os.path.join(cwd, 'src', 'sobits_vla_tools', 'pixi.toml'))
    candidates.append(os.path.join(cwd, 'sobits_vla_tools', 'pixi.toml'))
    candidates.append(os.path.join(cwd, 'pixi.toml'))
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)

    # 3) Last resort — legible error if the node ends up running without it.
    return 'pixi.toml'


def default_package_root(package_name: str, subdir: str, start_file: str) -> str:
    """Resolve <package_src>/<subdir>/ for `package_name`; thin wrapper over output_root."""
    return str(output_root(package_name, subdir, anchor_file=start_file))


def pixi_env_for(enable_gpu) -> str:
    """
    Map an ``enable_gpu`` launch argument to a pixi environment name.

    pixi.toml defines only `cpu` and `gpu`, so this is the single place launch
    files translate the boolean. Accepts a bool or the string a
    ``LaunchConfiguration.perform()`` yields ('true'/'1'/'yes' -> gpu).
    """
    if isinstance(enable_gpu, str):
        enable_gpu = enable_gpu.strip().lower() in ('1', 'true', 'yes', 'on')
    return 'gpu' if enable_gpu else 'cpu'


def pixi_prefix(pixi_env: str, manifest: str = '') -> str:
    """
    Return a Node `prefix=` string that runs the node inside a pixi env.

    ``pixi_env`` is 'cpu' or 'gpu' — usually from :func:`pixi_env_for`. Empty
    -> empty string (no prefix; node runs in the ambient interpreter). Pass the
    result as ``Node(prefix=pixi_prefix(...) or None)``.

    The prefix ends in ``python`` on purpose. launch prepends the prefix to the
    node executable, which is a colcon-generated console-script whose shebang is
    hardcoded to ``#!/usr/bin/python3`` (system Python, without the pixi deps).
    Executing it directly would ignore the pixi env and fail with
    ModuleNotFoundError. Prepending ``python`` makes the pixi env's interpreter
    run the script, so the shebang is bypassed and the env's packages are used.
    """
    env = (pixi_env or '').strip()
    if not env:
        return ''
    manifest = manifest or default_pixi_manifest()
    return f'pixi run --manifest-path {manifest} -e {env} python'


def resolve_pixi_env(context) -> str:
    """
    Resolve the pixi env for the current launch context.

    ``pixi_env`` (an explicit env name) wins when set; the literal 'none'
    disables the prefix; otherwise the ``enable_gpu`` boolean picks cpu/gpu
    via :func:`pixi_env_for`. Requires the launch file to have declared both
    the ``pixi_env`` and ``enable_gpu`` launch arguments (see
    :func:`pixi_launch_arguments`).
    """
    explicit = LaunchConfiguration('pixi_env').perform(context).strip()
    if explicit:
        return '' if explicit.lower() == 'none' else explicit
    return pixi_env_for(LaunchConfiguration('enable_gpu').perform(context))


def pixi_launch_arguments(default_pixi_manifest_value: str) -> list:
    """
    Return the enable_gpu/pixi_env/pixi_manifest DeclareLaunchArgument triple.

    Shared verbatim by every launch file that runs a node in a pixi env.
    ``default_pixi_manifest_value`` is normally the caller's own
    ``default_pixi_manifest()`` result, kept as a module-level constant so it
    is computed once at launch-description build time, not per-argument.
    """
    return [
        DeclareLaunchArgument(
            'enable_gpu',
            default_value='true',
            description=(
                'true -> run the node in the `gpu` pixi env (CUDA torch); '
                'false -> the `cpu` env. Set pixi_env:="" to skip the pixi '
                'prefix entirely and use the ambient interpreter.'
            ),
        ),
        DeclareLaunchArgument(
            'pixi_env',
            default_value='',
            description=(
                'Explicit pixi environment name, overriding enable_gpu. '
                'Empty (default) derives it from enable_gpu; "none" '
                'disables the pixi prefix.'
            ),
        ),
        DeclareLaunchArgument(
            'pixi_manifest',
            default_value=default_pixi_manifest_value,
            description='Path to pixi.toml (override for installed layouts).',
        ),
    ]
