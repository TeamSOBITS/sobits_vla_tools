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
Golden regression test.

Converts the synthetic fixture bag end-to-end and checks output shape
against a checked-in expected dict (R11-style, but runs every colcon
test instead of needing real recorded bags).
"""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from make_fixture_bag import EPISODE_NAME, generate, NUM_FRAMES  # noqa: E402

import pytest  # noqa: E402

# find_spec (not `import lerobot`) keeps this out of test_lerobot_seam's
# stray-import check -- lerobot imports are only allowed in the seam files.
_DEPS_AVAILABLE = (
    importlib.util.find_spec('rclpy') is not None
    and importlib.util.find_spec('lerobot') is not None
)

# colcon test runs ament_python packages under system Python (rclpy only);
# lerobot/pandas/torch live in the pixi env -- skip there, run under pixi.
skip_no_rclpy = pytest.mark.skipif(
    not _DEPS_AVAILABLE, reason='rclpy/lerobot not importable outside the pixi env.'
)

# Checked in for reviewer sign-off; regenerate by reading meta/info.json from
# a passing run if the pipeline's feature schema intentionally changes.
EXPECTED_FEATURES = {
    'action': {'dtype': 'float32', 'shape': [2], 'names': ['head_yaw_joint', 'head_pitch_joint']},
    'observation.state': {
        'dtype': 'float32', 'shape': [2], 'names': ['head_yaw_joint', 'head_pitch_joint'],
    },
    'observation.images.head_camera': {'dtype': 'video', 'shape': [3, 48, 64]},
    'timestamp': {'dtype': 'float32', 'shape': [1]},
    'frame_index': {'dtype': 'int64', 'shape': [1]},
    'episode_index': {'dtype': 'int64', 'shape': [1]},
    'index': {'dtype': 'int64', 'shape': [1]},
    'task_index': {'dtype': 'int64', 'shape': [1]},
}


def _run_conversion(subset_dir: Path, dataset_name: str, output_dir: Path):
    """Construct+spin RosbagConversionNode in-process with explicit overrides."""
    import rclpy
    from sobits_vla_rosbag_conversion.ros2bag_to_lerobotdataset import RosbagConversionNode

    ros_args = [
        '--ros-args',
        '-p', f'rosbag_directory:={subset_dir}',
        '-p', f'recorded_bags_meta_file:={subset_dir}/recorded_bags_meta.yaml',
        '-p', f'dataset_name:={dataset_name}',
        '-p', f'output_directory:={output_dir}',
        '-p', 'fps:=10',
        '-p', 'vcodec:=h264',
        '-p', 'sync_threshold:=0.2',
        '-p', 'downsample_tolerance:=0.02',
        '-p', 'push_to_hub:=false',
        '-p', 'hub_private:=false',
        '-p', 'overwrite:=true',
        '-p', 'use_relative_actions:=false',
        '-p', 'skip_static_threshold:=0.0',
        '-p', 'robot_descriptor_id:=sobit_light',
        '-p', 'exclude.groups:=[manipulator,end_effector]',
        '-p', 'exclude.cameras:=[hand_camera]',
        '-p', 'exclude.mobile_base:=true',
        '-p', 'cameras.primary:=head_camera',
    ]
    rclpy.init(args=['test_conversion_golden'] + ros_args)
    try:
        node = RosbagConversionNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, SystemExit):
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def _convert_once(tmp_path: Path, tag: str) -> Path:
    subset = tmp_path / f'bag_{tag}'
    out = tmp_path / f'out_{tag}'
    generate(subset)
    _run_conversion(subset, f'golden_{tag}', out)
    return out / f'golden_{tag}'


@skip_no_rclpy
def test_fixture_generation_is_deterministic(tmp_path):
    """Same generator run twice -> byte-identical mcap and meta yaml."""
    a = tmp_path / 'a'
    b = tmp_path / 'b'
    bag_a = generate(a)
    bag_b = generate(b)
    mcap_a = next(bag_a.glob('*.mcap'))
    mcap_b = next(bag_b.glob('*.mcap'))
    assert mcap_a.read_bytes() == mcap_b.read_bytes()
    meta_a = (a / 'recorded_bags_meta.yaml').read_text()
    meta_b = (b / 'recorded_bags_meta.yaml').read_text()
    assert meta_a == meta_b


@skip_no_rclpy
def test_conversion_golden(tmp_path):
    ds_root = _convert_once(tmp_path, 'run1')

    info = json.loads((ds_root / 'meta' / 'info.json').read_text())
    assert info['fps'] == 10
    assert info['robot_type'] == 'mobile_manipulator'
    for key, expected in EXPECTED_FEATURES.items():
        actual = info['features'][key]
        assert actual['dtype'] == expected['dtype'], key
        assert actual['shape'] == expected['shape'], key
        if 'names' in expected:
            assert actual['names'] == expected['names'], key

    stats = _load_yaml(ds_root / 'conversion_stats.yaml')
    assert stats['total_episodes'] == 1
    assert stats['total_frames'] == NUM_FRAMES
    assert stats['skipped_bags'] == []
    ep = stats['episodes'][0]
    assert ep['bag'] == EPISODE_NAME
    assert ep['frames'] == NUM_FRAMES
    assert ep['skipped_static'] == 0
    assert ep['skipped_tf'] == 0
    assert ep['skipped_img_decode'] == 0

    import pandas as pd

    df = pd.read_parquet(ds_root / 'data' / 'chunk-000' / 'file-000.parquet')
    assert len(df) == NUM_FRAMES
    assert len(df['action'].iloc[0]) == 2
    assert len(df['observation.state'].iloc[0]) == 2

    video = ds_root / 'videos' / 'observation.images.head_camera' / 'chunk-000' / 'file-000.mp4'
    assert video.is_file()
    nb_frames = _ffprobe_frame_count(video)
    assert nb_frames == NUM_FRAMES


@skip_no_rclpy
def test_conversion_golden_is_repeatable(tmp_path):
    """Two independent conversion runs over independently-generated fixtures agree."""
    ds1 = _convert_once(tmp_path, 'rep1')
    ds2 = _convert_once(tmp_path, 'rep2')

    stats1 = _load_yaml(ds1 / 'conversion_stats.yaml')
    stats2 = _load_yaml(ds2 / 'conversion_stats.yaml')
    for key in ('total_episodes', 'total_frames', 'skipped_bags', 'fps_warnings'):
        assert stats1[key] == stats2[key]
    ep1, ep2 = stats1['episodes'][0], stats2['episodes'][0]
    per_ep_keys = (
        'frames', 'skipped_static', 'skipped_tf',
        'skipped_img_decode', 'skipped_downsample',
    )
    for key in per_ep_keys:
        assert ep1[key] == ep2[key]

    import numpy as np
    import pandas as pd

    df1 = pd.read_parquet(ds1 / 'data' / 'chunk-000' / 'file-000.parquet')
    df2 = pd.read_parquet(ds2 / 'data' / 'chunk-000' / 'file-000.parquet')
    for col in ('action', 'observation.state'):
        a = np.stack(df1[col].values).astype(np.float64)
        b = np.stack(df2[col].values).astype(np.float64)
        assert np.allclose(a, b, atol=1e-6)


def _load_yaml(path: Path) -> dict:
    import yaml
    return yaml.safe_load(path.read_text())


def _ffprobe_frame_count(video: Path) -> int:
    out = subprocess.run(
        [
            'ffprobe', '-v', 'error', '-select_streams', 'v:0', '-count_frames',
            '-show_entries', 'stream=nb_read_frames', '-of', 'csv=p=0', str(video),
        ],
        capture_output=True, text=True, check=True,
    )
    return int(out.stdout.strip())


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
