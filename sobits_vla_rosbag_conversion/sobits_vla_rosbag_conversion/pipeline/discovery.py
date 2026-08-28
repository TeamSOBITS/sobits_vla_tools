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
Pure filesystem discovery of session/episode bag directories.

No rclpy import. Mirrors convert()'s directory-walk exactly, including its
asymmetry: candidate_bag_dirs (shape-sniffing) skips a missing group_dir,
while episode counting/conversion additionally retries with the bare
task_name (metadata from another machine may carry stale absolute paths).
"""

from dataclasses import dataclass, field
import os
from typing import Any, Dict, List, Optional, Tuple


def _is_bag_dir(path: str) -> bool:
    return os.path.isdir(path) and any(
        f.endswith('.db3') or f.endswith('.mcap') for f in os.listdir(path)
    )


def resolve_group_dir(
    meta_src: str, task_name: str, task_info: Dict[str, Any], retry_task_name: bool
) -> Optional[str]:
    """Resolve a task's bag group directory; None if unresolvable."""
    bag_group = task_info.get('bag_path', task_info.get('bag_dir', task_name))
    group_dir = bag_group if bag_group.startswith('/') else os.path.join(meta_src, bag_group)
    if os.path.isdir(group_dir):
        return group_dir
    if retry_task_name:
        fallback = os.path.join(meta_src, task_name)
        return fallback if os.path.isdir(fallback) else None
    return None


@dataclass
class TaskEpisodes:
    task_name: str
    task_info: Dict[str, Any]
    group_dir: str
    episode_dirs: List[str] = field(default_factory=list)


def discover_episodes(
    all_tasks: List[Tuple[str, Dict[str, Any]]], rosbag_directory: str
) -> List[TaskEpisodes]:
    """
    Resolve each task's group dir (with task_name fallback) and list its episode dirs.

    Skips tasks whose group_dir can't be resolved at all; callers wanting to
    warn about that (the conversion loop does) inspect the gap themselves.
    """
    out = []
    for task_name, task_info in all_tasks:
        meta_src = task_info.get('_meta_source_dir', rosbag_directory)
        group_dir = resolve_group_dir(meta_src, task_name, task_info, retry_task_name=True)
        if group_dir is None:
            continue
        episode_dirs = [
            os.path.join(group_dir, ep)
            for ep in sorted(os.listdir(group_dir))
            if _is_bag_dir(os.path.join(group_dir, ep))
        ]
        out.append(TaskEpisodes(task_name, task_info, group_dir, episode_dirs))
    return out


def candidate_bag_dirs(
    all_tasks: List[Tuple[str, Dict[str, Any]]], rosbag_directory: str
) -> List[str]:
    """
    Every episode dir across all tasks, for camera shape-sniffing.

    No task_name fallback (matches convert()'s original candidate-dir loop,
    which is more conservative than the counting/conversion loops below).
    """
    dirs = []
    for task_name, task_info in all_tasks:
        meta_src = task_info.get('_meta_source_dir', rosbag_directory)
        group_dir = resolve_group_dir(meta_src, task_name, task_info, retry_task_name=False)
        if group_dir is None:
            continue
        for ep in sorted(os.listdir(group_dir)):
            ep_path = os.path.join(group_dir, ep)
            if _is_bag_dir(ep_path):
                dirs.append(ep_path)
    return dirs
