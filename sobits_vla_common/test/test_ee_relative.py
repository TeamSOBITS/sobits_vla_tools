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

"""Unit tests for the batched SE(3) EE relative/absolute transforms in ee_relative.py."""

import math
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.ee_relative import (  # noqa: E402
    ee_groups_from_names, quat_to_rotvec, quat_to_rpy, rotvec_to_quat, rpy_to_quat,
    to_absolute_ee, to_relative_ee,
)
from sobits_vla_common.robot_descriptor import ee_action_features  # noqa: E402
import torch  # noqa: E402

REPS = ('rotvec', 'rpy', 'quat')


def _names(rotation):
    """[j0, ee.left.<axes>, gripper] layout: EE block embedded between other dims."""
    return ['j0'] + ee_action_features('left', rotation) + ['gripper']


def _rot_block(rot: Rotation, rotation: str) -> np.ndarray:
    if rotation == 'rotvec':
        return rot.as_rotvec()
    if rotation == 'rpy':
        return rot.as_euler('xyz')
    return rot.as_quat()


def _pose_vec(p, rot, rotation, extra=(0.3, 0.7)):
    """Assemble one [j0, ee..., gripper] row."""
    rows = [np.concatenate(([extra[0]], pi, _rot_block(ri, rotation), [extra[1]]))
            for pi, ri in zip(np.atleast_2d(p), rot)]
    return np.stack(rows)


def _random_case(rotation, B=4, T=5, seed=0):
    rng = np.random.default_rng(seed)
    names = _names(rotation)
    groups = ee_groups_from_names(names, names)
    obs_rot = Rotation.random(B, random_state=seed)
    obs_p = rng.normal(size=(B, 3))
    state = _pose_vec(obs_p, obs_rot, rotation)
    acts = np.stack([
        _pose_vec(rng.normal(size=(T, 3)), Rotation.random(T, random_state=seed + 1 + b), rotation)
        for b in range(B)])
    return names, groups, torch.tensor(acts), torch.tensor(state), obs_rot, obs_p


def _rot_of(block, rotation):
    b = np.asarray(block)
    if rotation == 'rotvec':
        return Rotation.from_rotvec(b)
    if rotation == 'rpy':
        return Rotation.from_euler('xyz', b)
    return Rotation.from_quat(b)


# --- helpers vs scipy ---


def test_rpy_quat_parity_with_scipy():
    rpy = np.array([[0.3, -0.4, 2.9], [-3.0, 1.2, -0.1], [0.0, 0.0, 0.0]])
    q = rpy_to_quat(torch.tensor(rpy)).numpy()
    ref = Rotation.from_euler('xyz', rpy).as_quat()
    assert np.allclose(np.sign(q[:, 3:]) * q, np.sign(ref[:, 3:]) * ref, atol=1e-12)
    assert np.allclose(quat_to_rpy(torch.tensor(ref)).numpy(), rpy, atol=1e-10)


def test_rotvec_quat_parity_with_scipy():
    rots = Rotation.random(20, random_state=3)
    rv = rots.as_rotvec()
    q = rotvec_to_quat(torch.tensor(rv)).numpy()
    assert np.allclose(q, rots.as_quat(canonical=True), atol=1e-10)
    assert np.allclose(quat_to_rotvec(torch.tensor(rots.as_quat())).numpy(), rv, atol=1e-10)
    # Zero and tiny rotations stay finite.
    tiny = torch.tensor([[0.0, 0.0, 0.0], [1e-9, 0.0, 0.0]], dtype=torch.float64)
    back = quat_to_rotvec(rotvec_to_quat(tiny))
    assert torch.allclose(back, tiny, atol=1e-15)


# --- grouping ---


def test_groups_parse_each_rep():
    for rotation in REPS:
        names = _names(rotation)
        (g,) = ee_groups_from_names(names, list(reversed(names)))
        assert g.arm == 'left' and g.rotation == rotation
        assert g.action_idx == tuple(range(1, 1 + len(ee_action_features('left', rotation))))
        assert [names[::-1][i] for i in g.state_idx] == ee_action_features('left', rotation)


def test_groups_arms_may_use_different_reps():
    names = ee_action_features('left', 'rotvec') + ee_action_features('right', 'quat')
    groups = ee_groups_from_names(names, names)
    assert [(g.arm, g.rotation) for g in groups] == [('left', 'rotvec'), ('right', 'quat')]


@pytest.mark.parametrize('names', [
    ['ee.left.x', 'ee.left.y', 'ee.left.z', 'ee.left.rx', 'ee.left.ry'],  # partial
    ee_action_features('left', 'rpy') + ['ee.left.rx'],  # mixed
    ['ee.left.x', 'ee.left.y', 'ee.left.z', 'ee.left.rx', 'ee.left.ry', 'ee.left.yaw'],
])
def test_groups_reject_partial_or_mixed(names):
    with pytest.raises(ValueError):
        ee_groups_from_names(names, names)


def test_groups_reject_missing_state():
    names = ee_action_features('left', 'rotvec')
    with pytest.raises(ValueError, match='state is missing'):
        ee_groups_from_names(names, names[:-1])


# --- transforms ---


@pytest.mark.parametrize('rotation', REPS)
def test_round_trip(rotation):
    _, groups, acts, state, _, _ = _random_case(rotation)
    rel = to_relative_ee(acts, state, groups)
    back = to_absolute_ee(rel, state, groups)
    if rotation == 'quat':
        # Compare as rotations: q and -q are the same pose.
        assert torch.allclose(back[..., :4], acts[..., :4], atol=1e-9)
        assert np.allclose(_rot_of(back[..., 4:8].reshape(-1, 4), 'quat').as_matrix(),
                           _rot_of(acts[..., 4:8].reshape(-1, 4), 'quat').as_matrix(), atol=1e-9)
        assert torch.allclose(back[..., 8], acts[..., 8])
    elif rotation == 'rpy':
        # Random targets may sit >pi from obs, so the unwrap moves them by 2*pi.
        diff = back - acts
        diff[..., 4:7] = torch.remainder(diff[..., 4:7] + math.pi, 2 * math.pi) - math.pi
        assert torch.allclose(diff, torch.zeros_like(diff), atol=1e-9)
    else:
        assert torch.allclose(back, acts, atol=1e-9)


@pytest.mark.parametrize('rotation', REPS)
def test_parity_with_scipy(rotation):
    _, groups, acts, state, obs_rot, obs_p = _random_case(rotation, seed=7)
    rel = to_relative_ee(acts, state, groups).numpy()
    n_rot = len(ee_action_features('left', rotation)) - 3
    for b in range(acts.shape[0]):
        a = acts[b].numpy()
        r_k = _rot_of(a[:, 4:4 + n_rot], rotation)
        exp_p = obs_rot[b].inv().apply(a[:, 1:4] - obs_p[b])
        exp_r = obs_rot[b].inv() * r_k
        assert np.allclose(rel[b, :, 1:4], exp_p, atol=1e-9)
        got = _rot_of(rel[b, :, 4:4 + n_rot], rotation)
        assert np.allclose(got.as_matrix(), exp_r.as_matrix(), atol=1e-9)
        # Non-EE dims untouched.
        assert np.allclose(rel[b, :, 0], a[:, 0]) and np.allclose(rel[b, :, -1], a[:, -1])
    if rotation == 'quat':
        assert (rel[..., 7] >= 0).all()


@pytest.mark.parametrize('rotation', REPS)
def test_identity_when_action_equals_obs(rotation):
    _, groups, _, state, _, _ = _random_case(rotation, seed=2)
    acts = state.unsqueeze(1).repeat(1, 3, 1)
    rel = to_relative_ee(acts, state, groups)
    ident = {'rotvec': [0, 0, 0], 'rpy': [0, 0, 0], 'quat': [0, 0, 0, 1]}[rotation]
    exp = torch.tensor([0.0, 0.0, 0.0] + ident, dtype=rel.dtype)
    assert torch.allclose(rel[..., 1:-1], exp.expand_as(rel[..., 1:-1]), atol=1e-12)


def test_body_frame_translation_known_value():
    names = ee_action_features('left', 'rotvec')
    groups = ee_groups_from_names(names, names)
    # Obs at (1, 0, 0) yawed +90 deg; target 1 m along world +y -> body +x.
    state = torch.tensor([[1.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2]], dtype=torch.float64)
    act = torch.tensor([[1.0, 1.0, 0.0, 0.0, 0.0, math.pi / 2]], dtype=torch.float64)
    rel = to_relative_ee(act, state, groups)
    assert torch.allclose(rel, torch.tensor([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
                                            dtype=torch.float64), atol=1e-12)


@pytest.mark.parametrize('rotation', REPS)
def test_large_rotation_se3_recovers_where_per_component_fails(rotation):
    names = ee_action_features('left', rotation)
    groups = ee_groups_from_names(names, names)
    obs = Rotation.from_euler('xyz', [0.4, -0.3, 2.8])
    turns = obs * Rotation.from_euler('xyz', [[0.0, 0.0, 1.9], [1.7, 0.2, 0.0], [0.3, 1.2, -2.0]])
    state = torch.tensor(np.concatenate(([0.1, 0.2, 0.3], _rot_block(obs, rotation)))[None])
    acts = torch.tensor(np.concatenate(
        (np.full((3, 3), 0.5), _rot_block(turns, rotation)), axis=1)[None])
    rel = to_relative_ee(acts, state, groups)
    back = to_absolute_ee(rel, state, groups)
    got = _rot_of(back[0, :, 3:].numpy(), rotation)
    assert np.allclose(got.as_matrix(), turns.as_matrix(), atol=1e-5)
    assert torch.allclose(back[..., :3], acts[..., :3], atol=1e-5)
    # Per-component rotation delta composed onto the same obs misses by a lot.
    naive = acts[0, :, 3:] - state[0, 3:]
    naive_rot = obs * _rot_of(naive.numpy(), rotation) if rotation != 'quat' else None
    if naive_rot is not None:
        err = (naive_rot.inv() * turns).magnitude()
        assert err.max() > 0.5


def test_unwrapped_obs_yaw_round_trips():
    names = ee_action_features('left', 'rpy')
    groups = ee_groups_from_names(names, names)
    state = torch.tensor([[0.0, 0.0, 0.0, 0.1, -0.2, 3.5]], dtype=torch.float64)
    yaws = torch.tensor([3.5, 3.9, 4.4, 5.0], dtype=torch.float64)
    acts = torch.zeros(1, 4, 6, dtype=torch.float64)
    acts[..., 3], acts[..., 4], acts[..., 5] = 0.1, -0.2, yaws
    back = to_absolute_ee(to_relative_ee(acts, state, groups), state, groups)
    assert torch.allclose(back, acts, atol=1e-9)


def test_quat_sign_invariance():
    names = ee_action_features('left', 'quat')
    groups = ee_groups_from_names(names, names)
    _, _, acts, state, _, _ = _random_case('quat', B=3, T=4, seed=5)
    acts, state = acts[..., 1:8], state[..., 1:8]
    flipped_state = state.clone()
    flipped_state[..., 3:] *= -1
    flipped_acts = acts.clone()
    flipped_acts[..., 3:] *= -1
    ref = to_relative_ee(acts, state, groups)
    for a, s in ((flipped_acts, state), (acts, flipped_state), (flipped_acts, flipped_state)):
        assert torch.allclose(to_relative_ee(a, s, groups), ref, atol=1e-12)
    back = to_absolute_ee(ref, flipped_state, groups)
    # Output lands on the obs quaternion's hemisphere.
    assert ((back[..., 3:] * flipped_state[:, None, 3:]).sum(-1) >= 0).all()


@pytest.mark.parametrize('rotation', REPS)
def test_2d_and_3d_shapes_agree(rotation):
    _, groups, acts, state, _, _ = _random_case(rotation)
    rel3 = to_relative_ee(acts, state, groups)
    rel2 = to_relative_ee(acts[:, 0], state, groups)
    assert rel2.shape == acts[:, 0].shape
    assert torch.allclose(rel2, rel3[:, 0])
    abs2 = to_absolute_ee(rel2, state, groups)
    assert abs2.shape == rel2.shape
    assert torch.allclose(abs2, to_absolute_ee(rel3, state, groups)[:, 0])


def test_dtype_and_device_follow_actions():
    _, groups, acts, state, _, _ = _random_case('rotvec')
    rel = to_relative_ee(acts.float(), state.double(), groups)
    assert rel.dtype == torch.float32 and rel.device == acts.device
    assert torch.allclose(rel.double(), to_relative_ee(acts, state, groups), atol=1e-5)
