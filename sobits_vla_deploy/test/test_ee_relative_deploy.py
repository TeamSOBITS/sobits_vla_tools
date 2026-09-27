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

"""C8: EE relative composition through the checkpoint pipeline at deploy."""

import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.ee_relative_processor import (  # noqa: E402
    EEAbsoluteActionsProcessorStep, insert_ee_relative_steps,
)
from sobits_vla_common.lerobot_adapter import (  # noqa: E402
    FeatureType, NormalizationMode, NormalizerProcessorStep, OBS_STATE,
    policy_action_to_transition, PolicyFeature, PolicyProcessorPipeline,
    transition_to_policy_action, UnnormalizerProcessorStep,
)
from sobits_vla_common.robot_descriptor import ee_action_features  # noqa: E402
from sobits_vla_deploy.deploy_node import LeRobotDeployNode  # noqa: E402
from sobits_vla_deploy.inference_engine import InferenceEngine  # noqa: E402
from sobits_vla_deploy.obs_builder import ObsBuilder  # noqa: E402
from sobits_vla_deploy.policy_loader import (  # noqa: E402
    _check_ee_relative, _dataset_action_names, _resolve_action_names,
)
import torch  # noqa: E402

EE_NAMES = ee_action_features('left', 'rotvec')
NAMES = ['j0'] + EE_NAMES + ['gripper']
D = len(NAMES)


def _identity_pipelines(names=NAMES):
    d = len(names)
    features = {
        OBS_STATE: PolicyFeature(type=FeatureType.STATE, shape=(d,)),
        'action': PolicyFeature(type=FeatureType.ACTION, shape=(d,)),
    }
    norm_map = {FeatureType.STATE: NormalizationMode.IDENTITY,
                FeatureType.ACTION: NormalizationMode.IDENTITY}
    pre = PolicyProcessorPipeline(
        steps=[NormalizerProcessorStep(features=features, norm_map=norm_map, stats={})],
        name='policy_preprocessor')
    post = PolicyProcessorPipeline(
        steps=[UnnormalizerProcessorStep(
            features={'action': features['action']}, norm_map=norm_map, stats={})],
        name='policy_postprocessor',
        to_transition=policy_action_to_transition,
        to_output=transition_to_policy_action)
    return pre, post


class _FakeRelativePolicy(torch.nn.Module):
    """Returns a fixed (1, T, D) relative chunk; records the state it was fed."""

    def __init__(self, chunk):
        super().__init__()
        self._dummy = torch.nn.Parameter(torch.zeros(1))
        self.chunk = chunk
        self.seen_state = None

    def predict_action_chunk(self, batch):
        self.seen_state = batch[OBS_STATE].clone()
        return self.chunk.clone()


def _engine(policy, pre, post):
    return InferenceEngine(
        policy=policy, model_device='cpu', model_use_amp=False, control_hz=10.0,
        actions_per_chunk=50, chunk_size_threshold=0.6, async_enabled=False,
        single_step_mode=False, rtc_enabled=False, rtc_inference_delay=0,
        preprocessor=pre, postprocessor=post, expected_state_dim=D,
        model_action_feature_names=NAMES, model_use_relative_actions=False,
        joint_features=['j0', 'gripper'], mobile_base_features=[],
        ee_features=EE_NAMES,
    )


class TestPipelineComposition:

    def test_chunk_composes_onto_obs_pose_through_large_turn(self):
        pre, post = _identity_pipelines()
        insert_ee_relative_steps(pre, post, NAMES, NAMES)

        r_obs = Rotation.from_rotvec([0.4, -1.1, 2.0])
        p_obs = np.array([0.45, 0.12, 0.83])
        state = np.concatenate(([0.7], p_obs, r_obs.as_rotvec(), [0.3])).astype(np.float32)

        # Relative chunk: up to ~150 deg about a tilted axis plus a small drift.
        T = 8
        axis = np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])
        angles = np.linspace(0.0, np.deg2rad(150.0), T)
        r_rel = Rotation.from_rotvec(axis[None] * angles[:, None])
        p_rel = np.stack([np.linspace(0, 0.1, T), np.linspace(0, -0.05, T),
                          np.linspace(0, 0.02, T)], axis=1)
        rel = np.concatenate(
            (np.full((T, 1), 0.2), p_rel, r_rel.as_rotvec(), np.full((T, 1), 0.9)), axis=1)
        policy = _FakeRelativePolicy(torch.tensor(rel, dtype=torch.float32)[None])

        steps, _raw, _delay = _engine(policy, pre, post)._predict_actions(
            {OBS_STATE: state.copy()}, state_vector={})

        assert torch.allclose(policy.seen_state[0], torch.from_numpy(state))
        assert len(steps) == T
        r_exp = r_obs * r_rel
        p_exp = p_obs[None] + r_obs.apply(p_rel)
        assert r_rel[-1].magnitude() > np.pi / 2
        for k, step in enumerate(steps):
            pos = np.array([step[f'ee.left.{a}'] for a in ('x', 'y', 'z')])
            rot = Rotation.from_rotvec([step[f'ee.left.{a}'] for a in ('rx', 'ry', 'rz')])
            assert np.allclose(pos, p_exp[k], atol=1e-5)
            assert (r_exp[k].inv() * rot).magnitude() < 1e-5
            # Joints / gripper stay absolute model output, untouched by the EE step.
            assert step['j0'] == pytest.approx(0.2, abs=1e-6)
            assert step['gripper'] == pytest.approx(0.9, abs=1e-6)


class TestLoaderEEChecks:

    def test_per_component_relative_on_ee_is_refused(self):
        cfg = SimpleNamespace(use_relative_actions=True, relative_exclude_joints=['gripper'])
        with pytest.raises(RuntimeError, match='per-component'):
            _check_ee_relative(cfg, None, None, NAMES)

    def test_per_component_relative_with_ee_excluded_is_allowed(self):
        cfg = SimpleNamespace(
            use_relative_actions=True, relative_exclude_joints=['gripper'] + EE_NAMES)
        assert _check_ee_relative(cfg, None, None, NAMES) is False

    def test_ee_step_without_preprocessor_is_refused(self):
        post = SimpleNamespace(steps=[EEAbsoluteActionsProcessorStep(enabled=True)])
        with pytest.raises(RuntimeError, match='no preprocessor'):
            _check_ee_relative(SimpleNamespace(), None, post, NAMES)

    def test_unpaired_ee_step_is_refused(self):
        pre, post = _identity_pipelines()
        insert_ee_relative_steps(pre, post, NAMES, NAMES)
        post.steps = [s for s in post.steps if not isinstance(s, EEAbsoluteActionsProcessorStep)]
        with pytest.raises(RuntimeError, match='unpaired'):
            _check_ee_relative(SimpleNamespace(), pre, post, NAMES)

    def test_paired_ee_steps_report_relative(self):
        pre, post = _identity_pipelines()
        insert_ee_relative_steps(pre, post, NAMES, NAMES)
        assert _check_ee_relative(SimpleNamespace(), pre, post, NAMES) is True

    def test_action_names_fall_back_to_ee_step_then_dataset(self):
        pre, post = _identity_pipelines()
        ds = SimpleNamespace(features={'action': {'names': ['a', *NAMES]}})
        assert _resolve_action_names(['c'], pre, ['a'], log=lambda m: None) == ['c']
        assert _resolve_action_names(
            None, pre, _dataset_action_names(ds), log=lambda m: None) == ['a', *NAMES]
        # Joint-only dataset names never override the positional mapping.
        assert _resolve_action_names(None, pre, ['a', 'b'], log=lambda m: None) is None
        insert_ee_relative_steps(pre, post, NAMES, NAMES)
        assert _resolve_action_names(None, pre, ['a'], log=lambda m: None) == NAMES
        assert _resolve_action_names(None, None, None, log=lambda m: None) is None
        assert _dataset_action_names(None) is None


def _check_node(**overrides):
    node = SimpleNamespace(
        _model_action_feature_names=NAMES, _action_space='ee', _model_repo_id='repo',
        _model_ee_rotation='rotvec', _ee_rotation='rotvec', _rtc_enabled=False,
        _model_ee_relative=False, _model_use_relative_actions=False,
    )
    for k, v in overrides.items():
        setattr(node, k, v)
    return node


class TestDeployNodeModelChecks:

    def test_matching_rotation_passes(self):
        LeRobotDeployNode._check_action_space_matches_model(_check_node())

    def test_rotation_mismatch_raises(self):
        node = _check_node(_ee_rotation='rpy')
        with pytest.raises(RuntimeError, match='ee_rotation'):
            LeRobotDeployNode._check_action_space_matches_model(node)

    def test_quat_refused(self):
        node = _check_node(
            _model_action_feature_names=ee_action_features('left', 'quat'),
            _model_ee_rotation='quat', _ee_rotation='quat')
        with pytest.raises(RuntimeError, match='rotvec and rpy only'):
            LeRobotDeployNode._check_action_space_matches_model(node)

    @pytest.mark.parametrize('flag', ['_model_ee_relative', '_model_use_relative_actions'])
    def test_rtc_with_relative_model_refused(self, flag):
        node = _check_node(_rtc_enabled=True, **{flag: True})
        with pytest.raises(RuntimeError, match='re-anchored'):
            LeRobotDeployNode._check_action_space_matches_model(node)

    def test_rtc_with_absolute_model_allowed(self):
        LeRobotDeployNode._check_action_space_matches_model(_check_node(_rtc_enabled=True))


class _FakeTfBuffer:
    def __init__(self, pos, quat):
        self.pos, self.quat = pos, quat

    def lookup_transform(self, target, source, stamp, timeout=None):
        tr = SimpleNamespace(x=self.pos[0], y=self.pos[1], z=self.pos[2])
        rot = SimpleNamespace(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        return SimpleNamespace(transform=SimpleNamespace(translation=tr, rotation=rot))


class TestObsBuilderRotation:

    def _builder(self, rotation):
        return ObsBuilder(
            joint_features=['j0'], mobile_base_features=[], camera_names=[],
            ee_state_specs=[('left', 'hand_left_end_effector_link', 'base_footprint')],
            ee_rotation=rotation)

    def test_rotvec_state_channels(self):
        builder = self._builder('rotvec')
        assert [k for k in builder.state_vector if k.startswith('ee.')] == EE_NAMES
        r = Rotation.from_rotvec([0.3, -2.5, 1.2])  # > 90 deg
        assert builder.refresh_ee_state(_FakeTfBuffer([0.4, 0.1, 0.9], r.as_quat()))
        sv = builder.state_vector
        assert np.allclose([sv['ee.left.x'], sv['ee.left.y'], sv['ee.left.z']],
                           [0.4, 0.1, 0.9], atol=1e-6)
        rv = [sv['ee.left.rx'], sv['ee.left.ry'], sv['ee.left.rz']]
        assert np.allclose(rv, r.as_rotvec(), atol=1e-5)

    def test_rpy_state_channels_unchanged(self):
        builder = self._builder('rpy')
        r = Rotation.from_euler('xyz', [0.2, -0.3, 1.0])
        assert builder.refresh_ee_state(_FakeTfBuffer([0.0, 0.0, 0.5], r.as_quat()))
        sv = builder.state_vector
        assert np.allclose([sv['ee.left.roll'], sv['ee.left.pitch'], sv['ee.left.yaw']],
                           [0.2, -0.3, 1.0], atol=1e-5)

    def test_quat_rejected(self):
        with pytest.raises(ValueError, match='rotvec or rpy'):
            self._builder('quat')
