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
Tests for robot.exclude.ee_poses rejection.

config_builder and preflight are plain functions over a flat params dict,
testable with no ROS node. config_builder.build_train_config imports
sobits_vla_common.lerobot_adapter unconditionally, so its tests are gated
on lerobot being importable (pixi env), same gate as
sobits_vla_rosbag_conversion/test/test_ee_actions.py.
"""

import importlib.util
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

skip_no_lerobot = pytest.mark.skipif(
    importlib.util.find_spec('lerobot') is None,
    reason='lerobot not importable outside the pixi env.',
)


def _base_params(**overrides):
    params = {
        'policy': 'smolvla',
        'robot.descriptor_id': '',
    }
    params.update(overrides)
    return params


@skip_no_lerobot
class TestConfigBuilderRejectsOldKey:

    def test_old_key_set_raises(self):
        from sobits_vla_training.config_builder import build_train_config
        params = _base_params(
            **{'robot.descriptor_id': 'ghost', 'robot.exclude.ee_poses': ['right']}
        )
        with pytest.raises(ValueError, match='robot.exclude.ee_poses was renamed'):
            build_train_config(params, output_dir=Path('/tmp/does-not-matter'))

    def test_old_key_empty_does_not_raise_from_deprecation_check(self):
        from sobits_vla_training.config_builder import build_train_config
        # No descriptor_id -> the descriptor branch (and its deprecation
        # check) never runs; this only proves an empty old key is inert.
        params = _base_params(**{'robot.exclude.ee_poses': []})
        build_train_config(params, output_dir=Path('/tmp/does-not-matter'))


class TestPreflightRejectsOldKey:

    def test_old_key_set_raises(self, monkeypatch):
        from sobits_vla_training import preflight

        monkeypatch.setattr(
            preflight, 'load_dataset_info',
            lambda repo_id: {'features': {'action': {'names': [], 'shape': []}}},
        )
        params = {
            'dataset.repo_id': 'some/dataset',
            'robot.descriptor_id': 'ghost',
            'robot.exclude.ee_poses': ['right'],
        }
        with pytest.raises(ValueError, match='robot.exclude.ee_poses was renamed'):
            preflight.run_preflight_checks(params)


@skip_no_lerobot
class TestEEActionArmsDerivation:
    """
    robot.ee_action_arms empty (default) now DERIVES the arm list.

    Uses the real sobit_home descriptor: arm_left/arm_right are both active
    and each has an ee_control entry, so excluding a group is what makes its
    ee_pose derive an EE action.
    """

    def test_joint_mode_derives_empty(self):
        from sobits_vla_training.config_builder import _ee_action_dim
        from sobits_vla_common.robot_descriptor import load_robot_descriptor
        desc = load_robot_descriptor('sobit_home')
        assert _ee_action_dim(desc, {}) == 0

    def test_excluded_group_derives_that_arms_ee_dim(self):
        from sobits_vla_training.config_builder import _ee_action_dim
        from sobits_vla_common.robot_descriptor import (
            ee_action_features, load_robot_descriptor,
        )
        desc = load_robot_descriptor('sobit_home').filtered(exclude_groups=['arm_left'])
        expected = len(ee_action_features('left'))
        assert _ee_action_dim(desc, {}) == expected

    def test_explicit_override_validated_against_derivation_rule(self):
        from sobits_vla_training.config_builder import _ee_action_dim
        from sobits_vla_common.robot_descriptor import load_robot_descriptor
        desc = load_robot_descriptor('sobit_home')  # arm_left still active
        with pytest.raises(ValueError, match='not excluded'):
            _ee_action_dim(desc, {'robot.ee_action_arms': ['left']})

    def test_explicit_override_matching_derivation_rule_passes(self):
        from sobits_vla_training.config_builder import _ee_action_dim
        from sobits_vla_common.robot_descriptor import (
            ee_action_features, load_robot_descriptor,
        )
        desc = load_robot_descriptor('sobit_home').filtered(exclude_groups=['arm_left'])
        expected = len(ee_action_features('left'))
        assert _ee_action_dim(desc, {'robot.ee_action_arms': ['left']}) == expected

    def test_preflight_expected_ee_actions_derives(self):
        from sobits_vla_training.preflight import _expected_ee_actions
        from sobits_vla_common.robot_descriptor import (
            ee_action_features, load_robot_descriptor,
        )
        desc = load_robot_descriptor('sobit_home').filtered(exclude_groups=['arm_left'])
        assert _expected_ee_actions(desc, {}) == ee_action_features('left')


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
