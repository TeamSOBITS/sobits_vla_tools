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
Offline TF tree for resolving frame chains from recorded /tf and /tf_static topics.

Uses binary search (bisect) for O(log n) temporal lookups on dynamic transforms.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class _DynamicEntry:
    """
    Time-sorted list of transforms for a single child frame.

    Attributes
    ----------
    stamps : list[int]
        List of nanosecond timestamps.
    parents : list[str]
        List of parent frame names.
    matrices : list[np.ndarray]
        List of 4x4 matrices.

    """

    stamps: list[int] = field(default_factory=list)
    parents: list[str] = field(default_factory=list)
    matrices: list[np.ndarray] = field(default_factory=list)

    def insert(self, stamp_ns: int, parent: str, mat: np.ndarray) -> None:
        """Insert a transform at stamp_ns."""
        idx = bisect.bisect_right(self.stamps, stamp_ns)
        self.stamps.insert(idx, stamp_ns)
        self.parents.insert(idx, parent)
        self.matrices.insert(idx, mat)

    def query(self, stamp_ns: int) -> tuple[str, np.ndarray] | None:
        """
        Return (parent, matrix) for the latest entry at or before *stamp_ns*.

        If stamp_ns precedes all recorded entries (e.g. first bag frame arrives before
        the first /tf message in message order), fall back to the earliest entry so that
        static-ish transforms (robot description, fixed joints) still resolve rather than
        causing a spurious lookup failure.
        """
        idx = bisect.bisect_right(self.stamps, stamp_ns) - 1
        if idx < 0:
            if self.stamps:
                return self.parents[0], self.matrices[0]
            return None
        return self.parents[idx], self.matrices[idx]


def _msg_to_mat(transform) -> np.ndarray:
    """Convert geometry_msgs/Transform to 4x4 homogeneous matrix."""
    q = transform.rotation
    t = transform.translation
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[0, 3], mat[1, 3], mat[2, 3] = t.x, t.y, t.z
    return mat


def mat_to_pose6d(mat: np.ndarray) -> np.ndarray:
    """Convert 4x4 homogeneous matrix to [x, y, z, roll, pitch, yaw] (float32)."""
    xyz = mat[:3, 3].astype(np.float32)
    rpy = Rotation.from_matrix(mat[:3, :3]).as_euler('xyz').astype(np.float32)
    return np.concatenate([xyz, rpy])


class OfflineTFTree:
    """
    Reads /tf and /tf_static messages and resolves arbitrary frame chains.

    Usage::

        tree = OfflineTFTree()
        for connection, timestamp, rawdata in reader.messages(connections=tf_conns):
            msg = reader.deserialize(rawdata, connection.msgtype)
            tree.ingest(msg, is_static=(connection.topic == '/tf_static'))

        mat = tree.resolve('base_link', 'hand_palm_link', stamp_ns)
        pose = mat_to_pose6d(mat)
    """

    def __init__(self) -> None:
        """Initialize empty static and dynamic transform caches."""
        self._static: dict[str, tuple[str, np.ndarray]] = {}
        self._dynamic: dict[str, _DynamicEntry] = {}

    def ingest(self, tf_msg, *, is_static: bool) -> None:
        """Add all transforms from a tf2_msgs/TFMessage."""
        for ts in tf_msg.transforms:
            child = ts.child_frame_id.lstrip('/')
            parent = ts.header.frame_id.lstrip('/')
            mat = _msg_to_mat(ts.transform)

            if is_static:
                self._static[child] = (parent, mat)
            else:
                entry = self._dynamic.setdefault(child, _DynamicEntry())
                entry.insert(
                    ts.header.stamp.sec * 1_000_000_000 + ts.header.stamp.nanosec,
                    parent, mat,
                )

    def _parent_and_mat(self, child: str, stamp_ns: int) -> tuple[str, np.ndarray] | None:
        """Best-effort lookup: prefer dynamic, fall back to static."""
        if child in self._dynamic:
            result = self._dynamic[child].query(stamp_ns)
            if result is not None:
                return result
        if child in self._static:
            return self._static[child]
        return None

    def resolve(
        self, target: str, source: str, stamp_ns: int, *, max_depth: int = 64
    ) -> np.ndarray | None:
        """
        Walk from *source* up the tree to *target*, returning the 4x4 transform.

        Returns ``None`` when the chain cannot be completed (missing frames).
        *max_depth* guards against cycles in malformed TF trees.
        """
        if source == target:
            return np.eye(4, dtype=np.float64)

        # Collect child→parent matrices along the chain
        chain: list[np.ndarray] = []
        current = source
        for _ in range(max_depth):
            if current == target:
                break
            result = self._parent_and_mat(current, stamp_ns)
            if result is None:
                return None
            parent, mat = result
            chain.append(mat)
            current = parent
        else:
            return None  # depth exceeded

        if not chain:
            return np.eye(4, dtype=np.float64)

        # Compose child->parent matrices root-to-leaf: this already is
        # T(target<-source), no inversion needed.
        composed = chain[-1]
        for mat in reversed(chain[:-1]):
            composed = composed @ mat
        return composed
