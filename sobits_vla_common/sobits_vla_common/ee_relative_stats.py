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
Normalization stats for relative actions with SE(3) EE groups.

Same output as lerobot.datasets.compute_stats.compute_relative_action_stats,
but applies the exact transform the preprocessor runs: per-component
subtraction on joint_mask dims, then to_relative_ee on the EE groups.
"""

from __future__ import annotations

import logging
from typing import Dict, Sequence

import numpy as np
from sobits_vla_common.ee_relative import EEGroup, to_relative_ee
from sobits_vla_common.lerobot_adapter import RunningQuantileStats
import torch

_BATCH_CHUNKS = 10_000


def _get_valid_chunk_starts(episode_indices: np.ndarray, chunk_size: int) -> np.ndarray:
    """Start indices whose chunk of chunk_size frames stays within one episode."""
    total = len(episode_indices)
    if total < chunk_size:
        return np.array([], dtype=np.int64)
    starts = np.arange(total - chunk_size + 1)
    return starts[episode_indices[starts] == episode_indices[starts + chunk_size - 1]]


def relative_chunks(
    actions: np.ndarray,
    states: np.ndarray,
    starts: np.ndarray,
    chunk_size: int,
    joint_mask: Sequence[bool],
    ee_groups: Sequence[EEGroup],
) -> np.ndarray:
    """(N, chunk_size, D) relative chunks anchored on states[starts]."""
    frame_idx = starts[:, None] + np.arange(chunk_size)[None, :]
    chunks = actions[frame_idx].astype(np.float64)
    anchor = states[starts].astype(np.float64)
    mask = np.asarray(joint_mask, dtype=np.float64)
    m = len(mask)
    chunks[:, :, :m] -= anchor[:, None, :m] * mask[None, None, :]
    if ee_groups:
        chunks = to_relative_ee(
            torch.from_numpy(chunks), torch.from_numpy(anchor), ee_groups).numpy()
    return chunks


def compute_relative_action_stats(
    actions: np.ndarray,
    states: np.ndarray,
    episode_index: np.ndarray,
    chunk_size: int,
    joint_mask: Sequence[bool],
    ee_groups: Sequence[EEGroup],
) -> Dict[str, np.ndarray]:
    """
    Per-dim stats over every in-episode action chunk, converted to relative.

    joint_mask may be shorter than the action dim (like lerobot's mask) and must
    be False on EE dims, else they would be converted twice. Returns the
    RunningQuantileStats dict: min, max, mean, std, count, q01..q99.
    """
    actions = np.asarray(actions, dtype=np.float32)
    states = np.asarray(states, dtype=np.float32)
    episode_index = np.asarray(episode_index)
    mask = [bool(v) for v in joint_mask]
    ee_dims = {i for g in ee_groups for i in g.action_idx}
    overlap = sorted(i for i in ee_dims if i < len(mask) and mask[i])
    if overlap:
        raise ValueError(f'joint_mask converts EE action dims {overlap}; exclude ee.* names')

    starts = _get_valid_chunk_starts(episode_index, chunk_size)
    if len(starts) == 0:
        raise RuntimeError(
            f'No valid chunks found (total_frames={len(episode_index)}, chunk_size={chunk_size})')

    running = RunningQuantileStats()
    for i in range(0, len(starts), _BATCH_CHUNKS):
        chunks = relative_chunks(
            actions, states, starts[i:i + _BATCH_CHUNKS], chunk_size, mask, ee_groups)
        running.update(chunks.reshape(-1, actions.shape[1]).astype(np.float32))
    stats = running.get_statistics()
    logging.info(
        f'Relative action stats ({len(starts)} chunks, chunk_size={chunk_size}): '
        f'joint_dims={sum(mask)}, ee_groups={[g.arm for g in ee_groups]}, '
        f'mean={np.abs(stats["mean"]).mean():.4f}, std={stats["std"].mean():.4f}')
    return stats
