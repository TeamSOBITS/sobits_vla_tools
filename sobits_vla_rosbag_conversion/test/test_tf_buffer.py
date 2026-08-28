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
Regression test for OfflineTFTree.resolve() convention (B1 / H3).

Builds a tiny static tree base -> link1 -> ee with known translations and
a known rotation, then checks resolve() returns T(target<-source) — not
its inverse.
"""

from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

# Package is not installed in the pixi envs -- make the test runnable from any cwd.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sobits_vla_rosbag_conversion.tf_buffer import OfflineTFTree  # noqa: E402


class _Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Transform:
    def __init__(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation


class _Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class _Header:
    def __init__(self, frame_id, stamp=None):
        self.frame_id = frame_id
        self.stamp = stamp or _Time()


class _TransformStamped:
    def __init__(self, parent, child, translation, rotation):
        self.header = _Header(parent)
        self.child_frame_id = child
        self.transform = _Transform(translation, rotation)


class _TFMessage:
    def __init__(self, transforms):
        self.transforms = transforms


def _build_tree() -> OfflineTFTree:
    """Build base -> link1 (+1m x, 90 deg z) -> ee (+0.5m x, no rotation)."""
    tree = OfflineTFTree()

    q1 = Rotation.from_euler('z', 90, degrees=True).as_quat()  # [x,y,z,w]
    base_to_link1 = _TransformStamped(
        'base', 'link1',
        _Vector3(1.0, 0.0, 0.0),
        _Quaternion(*q1),
    )
    link1_to_ee = _TransformStamped(
        'link1', 'ee',
        _Vector3(0.5, 0.0, 0.0),
        _Quaternion(0.0, 0.0, 0.0, 1.0),
    )
    tree.ingest(_TFMessage([base_to_link1, link1_to_ee]), is_static=True)
    return tree


def test_resolve_base_from_ee_matches_hand_computed():
    tree = _build_tree()
    mat = tree.resolve('base', 'ee', stamp_ns=0)

    assert mat is not None
    expected_translation = np.array([1.0, 0.5, 0.0])
    np.testing.assert_allclose(mat[:3, 3], expected_translation, atol=1e-9)

    expected_rpy = np.array([0.0, 0.0, 90.0])
    actual_rpy = Rotation.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=True)
    np.testing.assert_allclose(actual_rpy, expected_rpy, atol=1e-6)


def test_resolve_base_from_ee_equals_chained_single_hops():
    """T(base<-ee) must equal T(base<-link1) @ T(link1<-ee), not its inverse."""
    tree = _build_tree()
    mat_base_ee = tree.resolve('base', 'ee', stamp_ns=0)
    mat_base_link1 = tree.resolve('base', 'link1', stamp_ns=0)
    mat_link1_ee = tree.resolve('link1', 'ee', stamp_ns=0)

    assert mat_base_ee is not None
    assert mat_base_link1 is not None
    assert mat_link1_ee is not None
    np.testing.assert_allclose(
        mat_base_ee, mat_base_link1 @ mat_link1_ee, atol=1e-9
    )


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
