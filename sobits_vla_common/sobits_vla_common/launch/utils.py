"""
Shared launch helpers for sobits_vla_tools.

pixi manages the Python (non-ROS) dependencies of each package in its own
environment (see pixi.toml at the sobits_vla_tools root). A launch file can
therefore start a node in a *specific* pixi env — independent of whichever env
launched `ros2 launch` — by setting the Node `prefix=` to a `pixi run` command.
The spawned ros2 process (and the node) inherits that env's PATH/PYTHONPATH.

This is what lets a bringup launch in package A start package B's node in B's
own env instead of leaking A's env down the process tree.
"""

import os


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

    # 2) Non-symlink install: derive the workspace from colcon and look at the
    #    known source subpath <ws>/src/sobits_vla_tools/pixi.toml. Also probe the
    #    CWD subtree (ros2 launch usually runs from the workspace root).
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


def pixi_prefix(pixi_env: str, manifest: str = '') -> str:
    """
    Return a Node `prefix=` string that runs the node inside a pixi env.

    Empty ``pixi_env`` -> empty string (no prefix; node runs in the ambient
    interpreter). Pass the result as ``Node(prefix=pixi_prefix(...) or None)``.

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
