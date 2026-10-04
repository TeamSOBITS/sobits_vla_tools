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
Batched SE(3) conversion of EE pose actions to/from the observation frame.

UMI-style relative actions: A_k = inv(T_obs) . T_k, and back T_k = T_obs . A_k.
Conventions match geometry.py: quaternions (x, y, z, w), rpy = scipy 'xyz'
extrinsic (ROS RPY), rotvec = axis * angle. torch only, no lerobot import.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Sequence, Tuple

from sobits_vla_common.robot_descriptor import EE_ROTATION_AXES
import torch
from torch import Tensor


def quat_mul(a: Tensor, b: Tensor) -> Tensor:
    """Hamilton product a (x) b of (..., 4) quaternions (x, y, z, w)."""
    ax, ay, az, aw = a.unbind(-1)
    bx, by, bz, bw = b.unbind(-1)
    return torch.stack((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ), dim=-1)


def quat_conj(q: Tensor) -> Tensor:
    """Conjugate (= inverse for unit quaternions)."""
    return torch.cat((-q[..., :3], q[..., 3:]), dim=-1)


def quat_relative(q_from: Tensor, q_to: Tensor) -> Tensor:
    """Rotation from q_from to q_to: q_from^-1 (x) q_to."""
    return quat_mul(quat_conj(q_from), q_to)


def quat_rotate_vec(q: Tensor, v: Tensor) -> Tensor:
    """Rotate (..., 3) vectors v by unit quaternions q."""
    qv, w = q[..., :3], q[..., 3:]
    t = 2.0 * torch.cross(qv, v, dim=-1)
    return v + w * t + torch.cross(qv, t, dim=-1)


def quat_normalize(q: Tensor) -> Tensor:
    """Scale to unit norm (model outputs need not be unit)."""
    return q / q.norm(dim=-1, keepdim=True).clamp_min(1e-12)


def quat_shortest_arc(q: Tensor, q_ref: Tensor) -> Tensor:
    """Flip q where dot(q, q_ref) < 0 so it lies on q_ref's hemisphere."""
    dot = (q * q_ref).sum(dim=-1, keepdim=True)
    return torch.where(dot < 0.0, -q, q)


def quat_canonical(q: Tensor) -> Tensor:
    """Sign convention w >= 0."""
    return torch.where(q[..., 3:] < 0.0, -q, q)


def rpy_to_quat(rpy: Tensor) -> Tensor:
    """(..., 3) roll, pitch, yaw -> (..., 4) quaternion; same formula as geometry.py."""
    cr, sr = torch.cos(rpy[..., 0] * 0.5), torch.sin(rpy[..., 0] * 0.5)
    cp, sp = torch.cos(rpy[..., 1] * 0.5), torch.sin(rpy[..., 1] * 0.5)
    cy, sy = torch.cos(rpy[..., 2] * 0.5), torch.sin(rpy[..., 2] * 0.5)
    return torch.stack((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ), dim=-1)


def quat_to_rpy(q: Tensor) -> Tensor:
    """(..., 4) quaternion -> (..., 3) roll, pitch, yaw in (-pi, pi]."""
    x, y, z, w = q.unbind(-1)
    roll = torch.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = torch.asin((2.0 * (w * y - z * x)).clamp(-1.0, 1.0))
    yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return torch.stack((roll, pitch, yaw), dim=-1)


def rotvec_to_quat(rv: Tensor) -> Tensor:
    """(..., 3) axis * angle -> (..., 4) quaternion."""
    angle = rv.norm(dim=-1, keepdim=True)
    # sin(a/2)/a written via torch.sinc, finite at a = 0.
    scale = 0.5 * torch.sinc(angle / (2.0 * math.pi))
    return torch.cat((rv * scale, torch.cos(angle * 0.5)), dim=-1)


def quat_to_rotvec(q: Tensor) -> Tensor:
    """(..., 4) quaternion -> (..., 3) axis * angle, angle in [0, pi]."""
    q = quat_canonical(q)
    v, w = q[..., :3], q[..., 3:]
    n = v.norm(dim=-1, keepdim=True)
    angle = 2.0 * torch.atan2(n, w)
    # Small-angle limit of angle / n is 2 / w.
    small = n < 1e-7
    scale = torch.where(small, 2.0 / w.clamp_min(1e-12), angle / torch.where(small, 1.0, n))
    return v * scale


def wrap_to_near(x: Tensor, ref: Tensor) -> Tensor:
    """Shift x by multiples of 2*pi to lie within pi of ref."""
    two_pi = 2.0 * math.pi
    return x - two_pi * torch.round((x - ref) / two_pi)


def rot_to_quat(rot: Tensor, rotation: str) -> Tensor:
    """Convert a rotation block in rep 'rotvec' | 'rpy' | 'quat' to a unit quaternion."""
    if rotation == 'rotvec':
        return rotvec_to_quat(rot)
    if rotation == 'rpy':
        return rpy_to_quat(rot)
    if rotation == 'quat':
        return quat_normalize(rot)
    raise ValueError(f'unknown rotation {rotation!r}')


def quat_to_rot(q: Tensor, rotation: str) -> Tensor:
    """Convert a unit quaternion to rep 'rotvec' | 'rpy' | 'quat' (quat with w >= 0)."""
    if rotation == 'rotvec':
        return quat_to_rotvec(q)
    if rotation == 'rpy':
        return quat_to_rpy(q)
    if rotation == 'quat':
        return quat_canonical(q)
    raise ValueError(f'unknown rotation {rotation!r}')


@dataclass(frozen=True)
class EEGroup:
    """One arm's EE pose slots: position first, then rotation, in EE_ROTATION_AXES order."""

    arm: str
    rotation: str
    action_idx: Tuple[int, ...]
    state_idx: Tuple[int, ...]


def ee_groups_from_names(
    action_names: Sequence[str], state_names: Sequence[str]
) -> List[EEGroup]:
    """
    Group `ee.<arm>.<axis>` action names per arm and locate them in state.

    Each arm's axes must be exactly one EE_ROTATION_AXES set; the matching
    `ee.<arm>.<axis>` must exist in state_names. Raises ValueError otherwise.
    """
    per_arm: Dict[str, Dict[str, int]] = {}
    for i, name in enumerate(action_names):
        if not name.startswith('ee.'):
            continue
        arm, sep, axis = name[3:].rpartition('.')
        if not sep or not arm or not axis:
            raise ValueError(f'malformed EE feature name {name!r}, want ee.<arm>.<axis>')
        axes = per_arm.setdefault(arm, {})
        if axis in axes:
            raise ValueError(f'duplicate EE feature name {name!r}')
        axes[axis] = i

    state_pos = {n: i for i, n in enumerate(state_names)}
    groups = []
    for arm, axes in per_arm.items():
        rotation = next(
            (r for r, want in EE_ROTATION_AXES.items() if set(want) == set(axes)), None)
        if rotation is None:
            raise ValueError(
                f'EE arm {arm!r} has axes {sorted(axes)}; expected exactly one of '
                f'{sorted(EE_ROTATION_AXES)} axes {list(EE_ROTATION_AXES.values())}')
        order = EE_ROTATION_AXES[rotation]
        missing = [f'ee.{arm}.{ax}' for ax in order if f'ee.{arm}.{ax}' not in state_pos]
        if missing:
            raise ValueError(f'EE arm {arm!r}: state is missing {missing}')
        groups.append(EEGroup(
            arm=arm,
            rotation=rotation,
            action_idx=tuple(axes[ax] for ax in order),
            state_idx=tuple(state_pos[f'ee.{arm}.{ax}'] for ax in order),
        ))
    return groups


def _prepare(actions: Tensor, state: Tensor) -> Tuple[Tensor, Tensor, bool]:
    """Lift actions to (B, T, D), state to (B, S) on the actions' device/dtype."""
    squeeze = actions.ndim == 2
    if actions.ndim not in (2, 3):
        raise ValueError(f'actions must be (B, D) or (B, T, D), got {tuple(actions.shape)}')
    if state.ndim == 1:
        state = state.unsqueeze(0)
    state = state.to(device=actions.device, dtype=actions.dtype)
    return (actions.unsqueeze(1) if squeeze else actions), state, squeeze


def _obs_pose(state: Tensor, g: EEGroup) -> Tuple[Tensor, Tensor]:
    """Observation position (B, 1, 3) and unit quaternion (B, 1, 4) for one group."""
    s = state[:, list(g.state_idx)].unsqueeze(1)
    return s[..., :3], rot_to_quat(s[..., 3:], g.rotation)


def to_relative_ee(actions: Tensor, state: Tensor, groups: Sequence[EEGroup]) -> Tensor:
    """
    Absolute EE actions -> pose relative to the observation: A_k = inv(T_obs) . T_k.

    p_rel = R_o^T (p_k - p_o), q_rel = q_o^-1 (x) q_k, written in the group's rep.
    Non-EE dims pass through. actions (B, D) or (B, T, D), state (B, S).
    """
    acts, state, squeeze = _prepare(actions, state)
    out = acts.clone()
    for g in groups:
        idx = list(g.action_idx)
        a = acts[..., idx]
        p_o, q_o = _obs_pose(state, g)
        q_k = rot_to_quat(a[..., 3:], g.rotation)
        inv_o = quat_conj(q_o).expand_as(q_k)
        p_rel = quat_rotate_vec(inv_o, a[..., :3] - p_o)
        q_rel = quat_relative(q_o.expand_as(q_k), q_k)
        out[..., idx] = torch.cat((p_rel, quat_to_rot(q_rel, g.rotation)), dim=-1)
    return out.squeeze(1) if squeeze else out


def to_absolute_ee(rel: Tensor, state: Tensor, groups: Sequence[EEGroup]) -> Tensor:
    """
    Relative EE actions -> absolute: p = p_o + R_o p_rel, q = q_o (x) q_rel.

    rpy output is unwrapped against the obs rpy, then along the chunk; quat
    output is put on q_o's hemisphere. Non-EE dims pass through.
    """
    acts, state, squeeze = _prepare(rel, state)
    out = acts.clone()
    for g in groups:
        idx = list(g.action_idx)
        a = acts[..., idx]
        p_o, q_o = _obs_pose(state, g)
        q_rel = rot_to_quat(a[..., 3:], g.rotation)
        q_o = q_o.expand_as(q_rel)
        p = p_o + quat_rotate_vec(q_o, a[..., :3])
        q = quat_mul(q_o, q_rel)
        if g.rotation == 'rpy':
            rot = quat_to_rpy(q)
            obs_rpy = state[:, list(g.state_idx[3:])].unsqueeze(1)
            first = wrap_to_near(rot[:, :1], obs_rpy)
            steps = wrap_to_near(rot[:, 1:] - rot[:, :-1], torch.zeros_like(rot[:, 1:]))
            rot = torch.cat((first, first + torch.cumsum(steps, dim=1)), dim=1)
        elif g.rotation == 'quat':
            rot = quat_shortest_arc(quat_normalize(q), q_o)
        else:
            rot = quat_to_rotvec(q)
        out[..., idx] = torch.cat((p, rot), dim=-1)
    return out.squeeze(1) if squeeze else out
