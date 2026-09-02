#!/usr/bin/env python3

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

"""Visualize observation.ee_pose from a local LeRobot dataset."""

import argparse
import glob
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
import pandas as pd

matplotlib.use('Agg')


def load_dataset(dataset_root: Path) -> pd.DataFrame:
    """Load dataset from target path."""
    parquet_files = sorted(
        glob.glob(
            str(dataset_root / 'data' / '**' / '*.parquet'),
            recursive=True
        )
    )
    if not parquet_files:
        raise FileNotFoundError(f'No parquet files found under {dataset_root}/data/')
    return pd.concat([pd.read_parquet(f) for f in parquet_files], ignore_index=True)


def plot_ee(df: pd.DataFrame, ee_key: str, out_path: Path, max_episodes: int = 10):
    """Plot end-effector trajectories and time-series."""
    ee = np.stack(df[ee_key].values)           # (N, 6)
    episodes = df['episode_index'].values
    timestamps = df['timestamp'].values

    unique_eps = np.unique(episodes)[:max_episodes]
    cmap = plt.cm.tab10

    # ── Figure layout: 3D trajectory + 6 time-series ──────────────────────
    fig = plt.figure(figsize=(18, 10))
    ax3d = fig.add_subplot(2, 4, 1, projection='3d')
    ax_names = ['x (m)', 'y (m)', 'z (m)', 'roll (rad)', 'pitch (rad)', 'yaw (rad)']
    axes_ts = [fig.add_subplot(2, 4, i + 2) for i in range(6)]

    for idx, ep in enumerate(unique_eps):
        mask = episodes == ep
        t = timestamps[mask]
        t = t - t[0]                           # relative time per episode
        xyz = ee[mask, :3]
        color = cmap(idx / max(len(unique_eps) - 1, 1))

        ax3d.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                  color=color, alpha=0.7, linewidth=0.8, label=f'ep {ep}')
        ax3d.scatter(xyz[0, 0], xyz[0, 1], xyz[0, 2],
                     color=color, s=20, marker='o')       # start
        ax3d.scatter(xyz[-1, 0], xyz[-1, 1], xyz[-1, 2],
                     color=color, s=20, marker='x')       # end

        for i, ax in enumerate(axes_ts):
            ax.plot(t, ee[mask, i], color=color, alpha=0.7, linewidth=0.8)

    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title(f'EE 3D trajectory\n(first {len(unique_eps)} episodes)')
    ax3d.legend(fontsize=6, loc='upper left')

    for i, ax in enumerate(axes_ts):
        ax.set_title(ax_names[i])
        ax.set_xlabel('time (s)')
        ax.grid(True, alpha=0.3)

    fig.suptitle(f'End-effector pose — {ee_key}', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'Saved → {out_path}')

    # ── Per-axis distribution ──────────────────────────────────────────────
    fig2, axes2 = plt.subplots(2, 3, figsize=(14, 6))
    for i, (ax, name) in enumerate(zip(axes2.flat, ax_names)):
        ax.hist(ee[:, i], bins=60, color='steelblue', edgecolor='none', alpha=0.8)
        ax.set_title(name)
        ax.set_xlabel('value')
        ax.set_ylabel('count')
        ax.grid(True, alpha=0.3)
    fig2.suptitle(f'EE pose distribution — {ee_key}', fontsize=13, fontweight='bold')
    plt.tight_layout()
    dist_path = out_path.with_name(out_path.stem + '_distribution.png')
    plt.savefig(dist_path, dpi=150, bbox_inches='tight')
    print(f'Saved → {dist_path}')


def main():
    """Parse args and load/plot ee trajectory."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--dataset', type=Path,
        default=Path.home() / '.cache/huggingface/lerobot/sobit_home-pickup_the_block-gazebo',
        help='Path to local LeRobot dataset root')
    parser.add_argument(
        '--ee_key', default='observation.ee_pose',
        help='Column name for EE pose (e.g. observation.ee_pose.left)')
    parser.add_argument(
        '--max_episodes', type=int, default=10,
        help='Max episodes to overlay on trajectory plot')
    parser.add_argument(
        '--out', type=Path, default=Path('/tmp/ee_pose_viz.png'),
        help='Output PNG path')
    args = parser.parse_args()

    print(f'Loading dataset from {args.dataset} ...')
    df = load_dataset(args.dataset)
    print(f'  {len(df)} frames, {df["episode_index"].nunique()} episodes')

    if args.ee_key not in df.columns:
        available = [c for c in df.columns if 'ee_pose' in c]
        raise ValueError(f"Column '{args.ee_key}' not found. Available: {available}")

    plot_ee(df, args.ee_key, args.out, max_episodes=args.max_episodes)


if __name__ == '__main__':
    main()
