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
Tests for the preflight dataset action-convention checks.

Datasets are fake local roots under a monkeypatched HF_LEROBOT_HOME; the Hub
fallback is stubbed to fail so missing meta files read as absent.
"""

import json
import logging
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

REPO = 'team-sobits/fake-ee'
JOINTS = ['head_pan_joint', 'body_lift_joint']
EE_RPY = ['ee.left.x', 'ee.left.y', 'ee.left.z', 'ee.left.roll', 'ee.left.pitch', 'ee.left.yaw']
EE_ROTVEC = ['ee.left.x', 'ee.left.y', 'ee.left.z', 'ee.left.rx', 'ee.left.ry', 'ee.left.rz']


@pytest.fixture
def lerobot_home(tmp_path, monkeypatch):
    from sobits_vla_common import lerobot_adapter
    import huggingface_hub

    monkeypatch.setattr(lerobot_adapter, 'HF_LEROBOT_HOME', tmp_path, raising=False)

    def _no_hub(*args, **kwargs):
        raise FileNotFoundError('hub disabled in tests')
    monkeypatch.setattr(huggingface_hub, 'hf_hub_download', _no_hub)
    return tmp_path


def _make_dataset(home, names, convention=None, stats=None, conversion_stats=None):
    root = home / REPO
    meta = root / 'meta'
    meta.mkdir(parents=True)
    feat = {'names': names, 'shape': [len(names)]}
    info = {'features': {'action': dict(feat), 'observation.state': dict(feat)}}
    (meta / 'info.json').write_text(json.dumps(info))
    if convention is not None:
        sidecar = {'robot_info': {}, 'user_info': [], 'action_convention': convention}
        (meta / 'sobits_vla_info.json').write_text(json.dumps(sidecar))
    if stats is not None:
        (meta / 'stats.json').write_text(json.dumps(stats))
    if conversion_stats is not None:
        (root / 'conversion_stats.yaml').write_text(yaml.safe_dump(conversion_stats))
    return root


def _stats(names, action_mean, state_mean, state_std):
    n = len(names)
    return {
        'action': {'mean': action_mean, 'std': [0.1] * n},
        'observation.state': {'mean': state_mean, 'std': state_std},
    }


def _params(**overrides):
    params = {'dataset.repo_id': REPO, 'robot.descriptor_id': '', 'policy_overrides': {}}
    params.update(overrides)
    return params


def _run(params):
    from sobits_vla_training.preflight import run_preflight_checks
    run_preflight_checks(params)


class TestLoadMetaFile:

    def test_reads_sidecar_and_info(self, lerobot_home):
        from sobits_vla_training.preflight import load_dataset_info, load_dataset_meta_file
        _make_dataset(lerobot_home, JOINTS, convention={'action_mode': 'absolute'})
        assert load_dataset_info(REPO)['features']['action']['names'] == JOINTS
        sidecar = load_dataset_meta_file(REPO, 'sobits_vla_info.json')
        assert sidecar['action_convention'] == {'action_mode': 'absolute'}
        assert load_dataset_meta_file(REPO, 'stats.json') is None


class TestSidecarConvention:

    def test_absolute_ee_sidecar_passes(self, lerobot_home):
        conv = {'action_mode': 'absolute', 'ee_rotation': 'rotvec', 'ee_frames': {}}
        _make_dataset(lerobot_home, JOINTS + EE_ROTVEC, convention=conv)
        _run(_params(**{'robot.ee_rotation': 'rotvec'}))

    def test_relative_action_mode_fails(self, lerobot_home):
        conv = {'action_mode': 'relative', 'ee_rotation': 'rotvec'}
        _make_dataset(lerobot_home, JOINTS + EE_ROTVEC, convention=conv)
        with pytest.raises(RuntimeError, match='Reconvert'):
            _run(_params(**{'robot.ee_rotation': 'rotvec'}))

    def test_rotation_mismatch_fails(self, lerobot_home):
        conv = {'action_mode': 'absolute', 'ee_rotation': 'rpy', 'rpy_convention': 'xyz'}
        _make_dataset(lerobot_home, JOINTS + EE_RPY, convention=conv)
        with pytest.raises(RuntimeError, match='robot.ee_rotation'):
            _run(_params(**{'robot.ee_rotation': 'rotvec'}))

    def test_joint_only_absolute_passes(self, lerobot_home):
        _make_dataset(lerobot_home, JOINTS, convention={'action_mode': 'absolute'})
        _run(_params(policy_overrides={'use_relative_actions': True,
                                       'relative_exclude_joints': ['body_lift_joint']}))


class TestLegacyDataset:

    def test_legacy_rpy_matching_rotation_passes_with_warning(self, lerobot_home, caplog):
        names = JOINTS + EE_RPY
        # Absolute: action mean tracks state mean.
        stats = _stats(names, [0.3] * 8, [0.31] * 8, [0.1] * 8)
        _make_dataset(lerobot_home, names, stats=stats,
                      conversion_stats={'use_relative_actions': False})
        with caplog.at_level(logging.WARNING):
            _run(_params(**{'robot.ee_rotation': 'rpy'}))
        assert 'Legacy dataset without action_convention' in caplog.text

    def test_legacy_rotation_mismatch_fails(self, lerobot_home):
        names = JOINTS + EE_RPY
        _make_dataset(lerobot_home, names, stats=_stats(names, [0.3] * 8, [0.3] * 8, [0.1] * 8))
        with pytest.raises(RuntimeError, match="is 'rpy'"):
            _run(_params(**{'robot.ee_rotation': 'rotvec'}))

    def test_legacy_delta_stats_heuristic_fails(self, lerobot_home):
        names = JOINTS + EE_RPY
        # Deltas: action means near 0 while the arm sits ~0.6 m out.
        stats = _stats(names, [0.0] * 8, [0.0, 0.0, 0.6, 0.1, 0.8, 0.0, 0.0, 0.0], [0.1] * 8)
        _make_dataset(lerobot_home, names, stats=stats)
        with pytest.raises(RuntimeError, match=r'ee\.left\.x'):
            _run(_params(**{'robot.ee_rotation': 'rpy'}))

    def test_legacy_conversion_stats_relative_flag_fails(self, lerobot_home):
        names = JOINTS + EE_RPY
        _make_dataset(lerobot_home, names, conversion_stats={'use_relative_actions': True})
        with pytest.raises(RuntimeError, match='use_relative_actions=true'):
            _run(_params(**{'robot.ee_rotation': 'rpy'}))


class TestPerComponentEERelative:

    def test_refused_without_ee_relative_actions(self, lerobot_home):
        conv = {'action_mode': 'absolute', 'ee_rotation': 'rotvec'}
        _make_dataset(lerobot_home, JOINTS + EE_ROTVEC, convention=conv)
        params = _params(**{'robot.ee_rotation': 'rotvec'},
                         policy_overrides={'use_relative_actions': True})
        with pytest.raises(RuntimeError, match='per-component relative EE is refused'):
            _run(params)

    def test_allowed_with_ee_relative_actions(self, lerobot_home):
        conv = {'action_mode': 'absolute', 'ee_rotation': 'rotvec'}
        _make_dataset(lerobot_home, JOINTS + EE_ROTVEC, convention=conv)
        params = _params(**{'robot.ee_rotation': 'rotvec', 'robot.ee_relative_actions': True},
                         policy_overrides={'use_relative_actions': True})
        _run(params)


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))


class TestEEFrames:
    """_check_ee_frames: the dataset's EE frames must match the descriptor's current ones."""

    LEFT = {'source': 'hand_left_end_effector_link', 'target': 'body_lift_link'}

    def _check(self, frames):
        from sobits_vla_training.preflight import _check_ee_frames
        _check_ee_frames(REPO, frames, {'robot.descriptor_id': 'sobit_home'})

    def test_matching_frames_pass(self):
        self._check({'left': dict(self.LEFT)})

    def test_old_base_footprint_dataset_fails(self):
        with pytest.raises(RuntimeError, match='base_footprint.*body_lift_link'):
            self._check({'left': dict(self.LEFT, target='base_footprint')})

    def test_unknown_arm_and_no_descriptor_are_ignored(self):
        self._check({'ghost': dict(self.LEFT, target='base_footprint')})
        from sobits_vla_training.preflight import _check_ee_frames
        _check_ee_frames(REPO, {'left': dict(self.LEFT, target='x')}, {'robot.descriptor_id': ''})
