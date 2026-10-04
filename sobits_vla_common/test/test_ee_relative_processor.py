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

"""Pipeline tests for the SE(3) EE relative/absolute processor steps."""

import json
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_common.ee_relative import to_relative_ee  # noqa: E402
from sobits_vla_common.ee_relative_processor import (  # noqa: E402
    EE_ABSOLUTE_STEP_NAME, EE_RELATIVE_STEP_NAME, EEAbsoluteActionsProcessorStep,
    EERelativeActionsProcessorStep, insert_ee_relative_steps, reconnect_ee_relative_steps,
)
from sobits_vla_common.lerobot_adapter import (  # noqa: E402
    AbsoluteActionsProcessorStep, FeatureType, NormalizationMode, NormalizerProcessorStep,
    OBS_STATE, policy_action_to_transition, PolicyFeature, PolicyProcessorPipeline,
    RelativeActionsProcessorStep, transition_to_policy_action, UnnormalizerProcessorStep,
)
from sobits_vla_common.robot_descriptor import ee_action_features  # noqa: E402
import torch  # noqa: E402

NAMES = ['j0', 'j1'] + ee_action_features('left', 'rotvec') + ['gripper']
D = len(NAMES)
PRE_JSON = 'policy_preprocessor.json'
POST_JSON = 'policy_postprocessor.json'


def _pipelines():
    rng = np.random.default_rng(0)
    stats = {
        'action': {'mean': torch.tensor(rng.normal(size=D)),
                   'std': torch.tensor(rng.uniform(0.5, 2.0, size=D))},
        OBS_STATE: {'mean': torch.tensor(rng.normal(size=D)),
                    'std': torch.tensor(rng.uniform(0.5, 2.0, size=D))},
    }
    features = {
        OBS_STATE: PolicyFeature(type=FeatureType.STATE, shape=(D,)),
        'action': PolicyFeature(type=FeatureType.ACTION, shape=(D,)),
    }
    norm_map = {FeatureType.STATE: NormalizationMode.MEAN_STD,
                FeatureType.ACTION: NormalizationMode.MEAN_STD}
    pre = PolicyProcessorPipeline(
        steps=[NormalizerProcessorStep(features=features, norm_map=norm_map, stats=stats)],
        name='policy_preprocessor')
    post = PolicyProcessorPipeline(
        steps=[UnnormalizerProcessorStep(
            features={'action': features['action']}, norm_map=norm_map, stats=stats)],
        name='policy_postprocessor',
        to_transition=policy_action_to_transition,
        to_output=transition_to_policy_action)
    return pre, post


def _batch(B=3, T=6, seed=1):
    rng = np.random.default_rng(seed)

    def row(n, s):
        rot = Rotation.random(n, random_state=s).as_rotvec()
        return np.concatenate(
            (rng.normal(size=(n, 2)), rng.normal(size=(n, 3)), rot, rng.uniform(size=(n, 1))),
            axis=1)

    state = torch.tensor(row(B, seed), dtype=torch.float32)
    acts = torch.tensor(np.stack([row(T, seed + 10 + b) for b in range(B)]),
                        dtype=torch.float32)
    return state, acts


def _types(pipeline):
    return [type(s) for s in pipeline.steps]


def test_insert_places_steps_and_is_idempotent():
    pre, post = _pipelines()
    insert_ee_relative_steps(pre, post, NAMES, NAMES)
    assert _types(pre) == [EERelativeActionsProcessorStep, NormalizerProcessorStep]
    assert _types(post) == [UnnormalizerProcessorStep, EEAbsoluteActionsProcessorStep]
    rel, ab = insert_ee_relative_steps(pre, post, NAMES, NAMES)
    assert _types(pre) == [EERelativeActionsProcessorStep, NormalizerProcessorStep]
    assert _types(post) == [UnnormalizerProcessorStep, EEAbsoluteActionsProcessorStep]
    assert pre.steps[0] is rel and post.steps[1] is ab and ab.relative_step is rel


def test_insert_requires_normalizer_and_ee_names():
    pre, post = _pipelines()
    with pytest.raises(ValueError, match='NormalizerProcessorStep'):
        insert_ee_relative_steps(
            PolicyProcessorPipeline(steps=[], name='p'), post, NAMES, NAMES)
    with pytest.raises(ValueError, match='ee'):
        insert_ee_relative_steps(pre, post, ['j0', 'j1'], ['j0', 'j1'])


class GrootN17PackInputsStep:
    """Name-matched stand-in for GR00T's pack (normalize) step."""


class GrootN17ActionDecodeStep:
    """Name-matched stand-in for GR00T's decode (unnormalize) step."""


def test_insert_anchors_on_lerobot_relative_pair():
    pre, post = _pipelines()
    lr_rel = RelativeActionsProcessorStep(enabled=True)
    pre.steps = [lr_rel] + list(pre.steps)
    post.steps = list(post.steps) + [
        AbsoluteActionsProcessorStep(enabled=True, relative_step=lr_rel)]
    insert_ee_relative_steps(pre, post, NAMES, NAMES)
    assert _types(pre) == [RelativeActionsProcessorStep, EERelativeActionsProcessorStep,
                           NormalizerProcessorStep]
    assert _types(post) == [UnnormalizerProcessorStep, EEAbsoluteActionsProcessorStep,
                            AbsoluteActionsProcessorStep]


def test_insert_anchors_on_groot_pack_and_decode():
    pre = SimpleNamespace(steps=['rename', 'batch', GrootN17PackInputsStep(), 'vlm'])
    post = SimpleNamespace(steps=[GrootN17ActionDecodeStep(), 'to_cpu'])
    insert_ee_relative_steps(pre, post, NAMES, NAMES)
    assert isinstance(pre.steps[2], EERelativeActionsProcessorStep)
    assert isinstance(pre.steps[3], GrootN17PackInputsStep)
    assert isinstance(post.steps[1], EEAbsoluteActionsProcessorStep)
    assert post.steps[2] == 'to_cpu'


def test_relative_step_converts_ee_only():
    pre, post = _pipelines()
    rel, _ = insert_ee_relative_steps(pre, post, NAMES, NAMES)
    state, acts = _batch()
    out = rel({'observation': {OBS_STATE: state}, 'action': acts})
    # EnvTransition keys are TransitionKey enums; look the action up by value.
    action = next(v for k, v in out.items() if getattr(k, 'value', k) == 'action')
    assert torch.allclose(action, to_relative_ee(acts, state, rel.groups))
    assert torch.equal(rel.get_cached_state(), state)


def test_absolute_step_needs_pairing_and_state():
    step = EEAbsoluteActionsProcessorStep(enabled=True)
    with pytest.raises(RuntimeError, match='reconnect'):
        step({})
    step.relative_step = EERelativeActionsProcessorStep(
        enabled=True, action_names=NAMES, state_names=NAMES)
    with pytest.raises(RuntimeError, match='preprocessor'):
        step({})


def test_save_load_reconnect_round_trip(tmp_path):
    pre, post = _pipelines()
    insert_ee_relative_steps(pre, post, NAMES, NAMES)
    pre.save_pretrained(tmp_path, config_filename=PRE_JSON)
    post.save_pretrained(tmp_path, config_filename=POST_JSON)

    pre_cfg = json.loads((tmp_path / PRE_JSON).read_text())
    assert [s.get('registry_name') for s in pre_cfg['steps']] == [
        EE_RELATIVE_STEP_NAME, 'normalizer_processor']
    assert pre_cfg['steps'][0]['config'] == {
        'enabled': True, 'action_names': NAMES, 'state_names': NAMES}
    post_cfg = json.loads((tmp_path / POST_JSON).read_text())
    assert [s.get('registry_name') for s in post_cfg['steps']] == [
        'unnormalizer_processor', EE_ABSOLUTE_STEP_NAME]

    pre2 = PolicyProcessorPipeline.from_pretrained(tmp_path, config_filename=PRE_JSON)
    post2 = PolicyProcessorPipeline.from_pretrained(
        tmp_path, config_filename=POST_JSON,
        to_transition=policy_action_to_transition, to_output=transition_to_policy_action)
    assert post2.steps[1].relative_step is None
    reconnect_ee_relative_steps(pre2, post2)
    assert post2.steps[1].relative_step is pre2.steps[0]

    state, acts = _batch()
    batch = pre2({OBS_STATE: state, 'action': acts})
    # Normalized relative actions differ from the absolute input...
    assert not torch.allclose(batch['action'], acts, atol=1e-2)
    # ...and the postprocessor composes them back onto the cached obs pose.
    back = post2(batch['action'])
    assert torch.allclose(back, acts, atol=1e-4)
    back_step = post2(pre2({OBS_STATE: state, 'action': acts[:, 0]})['action'])
    assert torch.allclose(back_step, acts[:, 0], atol=1e-4)


def test_inference_caches_state_without_action():
    pre, post = _pipelines()
    rel, _ = insert_ee_relative_steps(pre, post, NAMES, NAMES)
    state, acts = _batch(B=1)
    pre({OBS_STATE: state})
    assert torch.equal(rel.get_cached_state(), state)
    rel_actions = to_relative_ee(acts, state, rel.groups)
    assert torch.allclose(post(post.steps[0]._normalize_action(rel_actions, inverse=False)),
                          acts, atol=1e-4)
