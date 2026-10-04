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

"""Tests for opt-in SE(3) EE relative actions wiring (config_builder + lerobot_train patches)."""

import importlib
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('lerobot') is None,
    reason='lerobot not importable outside the pixi env.',
)


class _Stop(Exception):
    pass


def _ee_names():
    from sobits_vla_common.robot_descriptor import ee_action_features
    return ee_action_features('left', 'rotvec')


def _policy_overrides(monkeypatch, **params):
    """policy_overrides that build_train_config hands to make_policy_config."""
    from sobits_vla_training import config_builder

    captured = {}

    def fake_make_policy_config(policy_type, overrides, device):
        captured.update(overrides)
        raise _Stop

    monkeypatch.setattr(config_builder, 'make_policy_config', fake_make_policy_config)
    base = {
        'policy': 'pi05',
        'num_gpus': 0,
        'robot.descriptor_id': 'sobit_home',
        'robot.exclude.groups': ['arm_left', 'arm_right', 'hand_right'],
        'robot.exclude.cameras': ['hand_right_camera'],
        'robot.exclude.ee': ['right'],
        'dataset.repo_id': 'x/y',
    }
    base.update(params)
    with pytest.raises(_Stop):
        config_builder.build_train_config(base, output_dir=Path('/tmp/does-not-matter'))
    return captured


class TestConfigBuilderExcludesEE:

    def test_both_flags_append_ee_names(self, monkeypatch):
        po = _policy_overrides(monkeypatch, **{
            'robot.ee_relative_actions': True,
            'policy_overrides': {'use_relative_actions': True}})
        exclude = po['relative_exclude_joints']
        assert exclude[-6:] == _ee_names()
        assert len(exclude) > 6  # descriptor-derived base/gripper names kept

    def test_explicit_exclude_kept_and_extended(self, monkeypatch):
        po = _policy_overrides(monkeypatch, **{
            'robot.ee_relative_actions': True,
            'policy_overrides': {'use_relative_actions': True,
                                 'relative_exclude_joints': ['gripper']}})
        assert po['relative_exclude_joints'] == ['gripper'] + _ee_names()

    @pytest.mark.parametrize('ee_rel,joint_rel', [(True, False), (False, True)])
    def test_single_flag_leaves_ee_convertible(self, monkeypatch, ee_rel, joint_rel):
        po = _policy_overrides(monkeypatch, **{
            'robot.ee_relative_actions': ee_rel,
            'policy_overrides': {'use_relative_actions': joint_rel}})
        exclude = po.get('relative_exclude_joints') or []
        assert not any(n in exclude for n in _ee_names())

    def test_rotvec_accepted(self, monkeypatch):
        po = _policy_overrides(monkeypatch, **{'robot.ee_rotation': 'rotvec'})
        assert po['max_action_dim'] >= 32


NAMES = ['j0', 'j1'] + ['ee.left.' + a for a in ('x', 'y', 'z', 'rx', 'ry', 'rz')] + ['gripper']
D = len(NAMES)
CHUNK = 4


def _frames(n_eps=3, ep_len=12, seed=0):
    from scipy.spatial.transform import Rotation

    rng = np.random.default_rng(seed)
    n = n_eps * ep_len

    def rows(s):
        return np.concatenate((rng.normal(size=(n, 2)), rng.normal(size=(n, 3)),
                               Rotation.random(n, random_state=s).as_rotvec(),
                               rng.uniform(size=(n, 1))), axis=1).astype(np.float32)

    return {'action': rows(seed + 1), 'observation.state': rows(seed + 2),
            'episode_index': np.repeat(np.arange(n_eps), ep_len)}


def _dataset(frames):
    abs_stats = {'mean': np.zeros(D), 'std': np.ones(D)}
    return SimpleNamespace(hf_dataset=frames,
                           meta=SimpleNamespace(stats={'action': dict(abs_stats)}))


def _spec(**kw):
    spec = {'ee_relative': True, 'joint_relative': True,
            'joint_exclude': ['gripper'] + NAMES[2:8],
            'action_names': NAMES, 'state_names': NAMES, 'ee_rotation': 'rotvec'}
    spec.update(kw)
    return spec


class _PackStep:
    def __init__(self, relative):
        self.relative = relative

    def _uses_relative_action_groups(self):
        return self.relative


@pytest.fixture
def lt(monkeypatch):
    """lerobot_train with stubbed dataset/processor factories (restored after the test)."""
    # Via importlib: test_lerobot_seam allows `import lerobot` only in the seam modules.
    lt = importlib.import_module('lerobot.scripts.lerobot_train')
    from sobits_vla_common.lerobot_adapter import (
        AbsoluteActionsProcessorStep, RelativeActionsProcessorStep,
    )

    frames = _frames()
    state = SimpleNamespace(train=_dataset(frames), eval=_dataset(frames), calls=0)

    def fake_datasets(cfg):
        return state.train, state.eval

    def fake_processors(*args, **kwargs):
        state.calls += 1
        rel = RelativeActionsProcessorStep(enabled=True)
        pre = SimpleNamespace(steps=[rel] + state.extra_pre)
        post = SimpleNamespace(
            steps=[AbsoluteActionsProcessorStep(enabled=True, relative_step=rel)]
            + state.extra_post)
        return pre, post

    state.extra_pre, state.extra_post = [], []
    state.frames = frames
    monkeypatch.setattr(lt, 'make_train_eval_datasets', fake_datasets)
    monkeypatch.setattr(lt, 'make_pre_post_processors', fake_processors)
    state.module = lt
    return state


def _cfg():
    return SimpleNamespace(policy=SimpleNamespace(action_delta_indices=list(range(CHUNK))))


class TestInstallEERelativeTraining:

    def test_stats_replaced_on_train_and_eval(self, lt):
        from sobits_vla_common.ee_relative import ee_groups_from_names
        from sobits_vla_common.ee_relative_stats import compute_relative_action_stats
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        install_ee_relative_training(_spec())
        train_ds, eval_ds = lt.module.make_train_eval_datasets(_cfg())
        f = lt.frames
        mask = [True, True] + [False] * 7
        want = compute_relative_action_stats(
            f['action'], f['observation.state'], f['episode_index'], CHUNK, mask,
            ee_groups_from_names(NAMES, NAMES))
        for ds in (train_ds, eval_ds):
            np.testing.assert_allclose(ds.meta.stats['action']['mean'], want['mean'], atol=1e-6)
            np.testing.assert_allclose(ds.meta.stats['action']['q99'], want['q99'], atol=1e-6)

    def test_joint_only_mask_skips_ee_groups(self, lt):
        from sobits_vla_common.ee_relative_stats import compute_relative_action_stats
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        install_ee_relative_training(_spec(ee_relative=False, joint_exclude=['gripper']))
        train_ds, _ = lt.module.make_train_eval_datasets(_cfg())
        f = lt.frames
        want = compute_relative_action_stats(
            f['action'], f['observation.state'], f['episode_index'], CHUNK,
            [True] * 8 + [False], [])
        np.testing.assert_allclose(train_ds.meta.stats['action']['mean'], want['mean'], atol=1e-6)

    def test_steps_inserted_next_to_lerobot_pair(self, lt):
        from sobits_vla_common.ee_relative_processor import (
            EEAbsoluteActionsProcessorStep, EERelativeActionsProcessorStep,
        )
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        install_ee_relative_training(_spec())
        pre, post = lt.module.make_pre_post_processors(policy_cfg=None)
        assert isinstance(pre.steps[1], EERelativeActionsProcessorStep)
        assert isinstance(post.steps[0], EEAbsoluteActionsProcessorStep)
        assert post.steps[0].relative_step is pre.steps[1]

    def test_reinstall_is_idempotent(self, lt):
        from sobits_vla_common.ee_relative_processor import EERelativeActionsProcessorStep
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        install_ee_relative_training(_spec())
        first = lt.module.make_pre_post_processors
        install_ee_relative_training(_spec())
        assert lt.module.make_pre_post_processors.__wrapped__ is first.__wrapped__
        pre, post = lt.module.make_pre_post_processors()
        assert lt.calls == 1
        assert sum(isinstance(s, EERelativeActionsProcessorStep) for s in pre.steps) == 1
        stats1 = lt.module.make_train_eval_datasets(_cfg())[0].meta.stats['action']['mean']
        stats2 = lt.module.make_train_eval_datasets(_cfg())[0].meta.stats['action']['mean']
        np.testing.assert_allclose(stats1, stats2)

    def test_off_leaves_everything_alone(self, lt):
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        install_ee_relative_training(_spec(ee_relative=False, joint_relative=False))
        train_ds, _ = lt.module.make_train_eval_datasets(_cfg())
        np.testing.assert_array_equal(train_ds.meta.stats['action']['mean'], np.zeros(D))
        pre, post = lt.module.make_pre_post_processors()
        assert len(pre.steps) == 1 and len(post.steps) == 1

    def test_lerobot_mask_covering_ee_refused(self):
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        with pytest.raises(RuntimeError, match='relative_exclude_joints'):
            install_ee_relative_training(_spec(joint_exclude=['gripper']))

    def test_rotation_mismatch_refused(self):
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        with pytest.raises(RuntimeError, match='rotation'):
            install_ee_relative_training(_spec(ee_rotation='rpy'))

    @pytest.mark.parametrize('where', ['pack', 'decode'])
    def test_groot_native_relative_refused(self, lt, where):
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        if where == 'pack':
            lt.extra_pre = [_PackStep(relative=True)]
        else:
            lt.extra_post = [SimpleNamespace(use_relative_action=True)]
        install_ee_relative_training(_spec())
        with pytest.raises(RuntimeError, match='GR00T native'):
            lt.module.make_pre_post_processors()

    def test_groot_non_native_allowed(self, lt):
        from sobits_vla_common.lerobot_compat import install_ee_relative_training

        lt.extra_pre = [_PackStep(relative=False)]
        lt.extra_post = [SimpleNamespace(use_relative_action=False)]
        install_ee_relative_training(_spec())
        pre, _ = lt.module.make_pre_post_processors()
        assert len(pre.steps) == 3


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))


class TestRelativeTrainingSpec:

    def _info(self):
        return {'features': {'action': {'names': ['j1', 'ee.left.x']},
                             'observation.state': {'names': ['j1', 'ee.left.x']}}}

    def test_nothing_relative_returns_none(self):
        from sobits_vla_training.relative_training import relative_training_spec
        policy = SimpleNamespace(use_relative_actions=False)
        assert relative_training_spec({}, policy, self._info(), lambda m: None) is None

    def test_ee_relative_without_info_raises(self):
        from sobits_vla_training.relative_training import relative_training_spec
        policy = SimpleNamespace(use_relative_actions=False)
        with pytest.raises(RuntimeError, match='meta/info.json'):
            relative_training_spec(
                {'robot.ee_relative_actions': True}, policy, None, lambda m: None)

    def test_joint_relative_without_info_warns_and_returns_none(self):
        from sobits_vla_training.relative_training import relative_training_spec
        warnings = []
        policy = SimpleNamespace(use_relative_actions=True, relative_exclude_joints=['j1'])
        assert relative_training_spec({}, policy, None, warnings.append) is None
        assert warnings and 'absolute-space' in warnings[0]

    def test_spec_carries_names_mask_and_rotation(self):
        from sobits_vla_training.relative_training import relative_training_spec
        policy = SimpleNamespace(use_relative_actions=True, relative_exclude_joints=['j1'])
        spec = relative_training_spec(
            {'robot.ee_relative_actions': True, 'robot.ee_rotation': 'rpy'},
            policy, self._info(), lambda m: None)
        assert spec == {
            'ee_relative': True, 'joint_relative': True, 'joint_exclude': ['j1'],
            'action_names': ['j1', 'ee.left.x'], 'state_names': ['j1', 'ee.left.x'],
            'ee_rotation': 'rpy',
        }
