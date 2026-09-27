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

"""Hand-computed checks for the SE(3)-aware relative action stats."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.ee_relative import ee_groups_from_names  # noqa: E402
from sobits_vla_common.ee_relative_stats import (  # noqa: E402
    _get_valid_chunk_starts, compute_relative_action_stats,
)
from sobits_vla_common.robot_descriptor import ee_action_features  # noqa: E402

NAMES = ['j0'] + ee_action_features('left', 'rotvec') + ['gripper']
GROUPS = ee_groups_from_names(NAMES, NAMES)
JOINT_MASK = [True] + [False] * 6  # gripper left out: shorter mask = absolute
H = math.pi / 2


def _row(j, p, yaw, g=0.0):
    return [j, *p, 0.0, 0.0, yaw, g]


# Episode 0 = frames 0-2, episode 1 = frames 3-4; poses only yaw about z.
STATES = np.array([
    _row(0, (0, 0, 0), 0.0),
    _row(1, (1, 0, 0), H),
    _row(3, (0, 0, 0), 0.0),
    _row(10, (0, 0, 0), H),
    _row(0, (0, 0, 0), 0.0),
])
ACTIONS = np.array([
    _row(1, (1, 0, 0), 0.5, 0.0),
    _row(3, (1, 2, 0), H, 0.1),
    _row(4, (0, 0, 1), -0.5, 0.2),
    _row(10, (0, 1, 0), H, 0.3),
    _row(12, (-1, 0, 0), H, 0.4),
])
EPISODES = np.array([0, 0, 0, 1, 1])

# chunk_size=2 -> starts 0, 1, 3 (2 straddles episodes). Obs yaw +90 maps (x, y) -> (y, -x).
EXPECTED = np.array([
    _row(1, (1, 0, 0), 0.5, 0.0),       # start 0, frame 0
    _row(3, (1, 2, 0), H, 0.1),         # start 0, frame 1
    _row(2, (2, 0, 0), 0.0, 0.1),       # start 1, frame 1
    _row(3, (0, 1, 1), -0.5 - H, 0.2),  # start 1, frame 2
    _row(0, (1, 0, 0), 0.0, 0.3),       # start 3, frame 3
    _row(2, (0, 1, 0), 0.0, 0.4),       # start 3, frame 4
])


def test_valid_chunk_starts():
    assert _get_valid_chunk_starts(EPISODES, 2).tolist() == [0, 1, 3]
    assert _get_valid_chunk_starts(EPISODES, 3).tolist() == [0]
    assert _get_valid_chunk_starts(EPISODES, 6).tolist() == []


def test_stats_match_hand_computed():
    stats = compute_relative_action_stats(ACTIONS, STATES, EPISODES, 2, JOINT_MASK, GROUPS)
    assert stats['count'].tolist() == [6]
    assert np.allclose(stats['min'], EXPECTED.min(0), atol=1e-6)
    assert np.allclose(stats['max'], EXPECTED.max(0), atol=1e-6)
    assert np.allclose(stats['mean'], EXPECTED.mean(0), atol=1e-6)
    assert np.allclose(stats['std'], EXPECTED.std(0), atol=1e-5)
    for key in ('q01', 'q10', 'q50', 'q90', 'q99'):
        assert stats[key].shape == (len(NAMES),)


def test_rejects_joint_mask_over_ee_dims():
    with pytest.raises(ValueError, match='EE action dims'):
        compute_relative_action_stats(ACTIONS, STATES, EPISODES, 2, [True] * 8, GROUPS)


def test_no_valid_chunks_raises():
    with pytest.raises(RuntimeError):
        compute_relative_action_stats(ACTIONS, STATES, EPISODES, 9, JOINT_MASK, GROUPS)
