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

import dataclasses
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy.fake_policy import (  # noqa: E402
    fake_bundle, FakeBundle, FakePolicy,
)

KEYS = ['joint_a', 'joint_b', 'x.vel', 'theta.vel']


def test_predict_holds_measured_and_zeroes_velocity():
    policy = FakePolicy(KEYS, 4, latency_s=0.0)
    steps, raw, delay = policy.predict({}, {'joint_a': 0.5, 'joint_b': -1.0, 'x.vel': 9.0})
    assert len(steps) == 4 and raw is None and delay == 0
    assert steps[0] == {'joint_a': 0.5, 'joint_b': -1.0, 'x.vel': 0.0, 'theta.vel': 0.0}


def test_unmeasured_joint_defaults_to_zero():
    steps, _, _ = FakePolicy(KEYS, 1, latency_s=0.0).predict({}, {})
    assert steps == [{'joint_a': 0.0, 'joint_b': 0.0, 'x.vel': 0.0, 'theta.vel': 0.0}]


def test_steps_are_independent_dicts():
    steps, _, _ = FakePolicy(KEYS, 2, latency_s=0.0).predict({}, {})
    steps[0]['joint_a'] = 1.0
    assert steps[1]['joint_a'] == 0.0


def test_chunk_length_floor_is_one():
    steps, _, _ = FakePolicy(KEYS, 0, latency_s=0.0).predict({}, {})
    assert len(steps) == 1


def test_latency_is_simulated():
    start = time.monotonic()
    FakePolicy(KEYS, 1, latency_s=0.05).predict({}, {})
    assert time.monotonic() - start >= 0.045


def test_bundle_matches_policy_bundle_fields():
    names = [f.name for f in dataclasses.fields(FakeBundle)]
    assert names == [
        'policy', 'rtc_enabled', 'model_action_feature_names', 'model_use_relative_actions',
        'model_ee_relative', 'model_ee_rotation', 'expected_state_dim', 'preprocessor',
        'postprocessor']
    bundle = fake_bundle(KEYS, 3)
    assert isinstance(bundle.policy, FakePolicy) and bundle.policy.actions_per_chunk == 3
    assert bundle.model_action_feature_names == KEYS
    assert not bundle.rtc_enabled and not bundle.model_ee_relative
    assert bundle.model_ee_rotation == ''


def test_fields_match_real_policy_bundle():
    pytest.importorskip('torch')
    from sobits_vla_deploy.policy_loader import PolicyBundle
    assert [f.name for f in dataclasses.fields(FakeBundle)] == [
        f.name for f in dataclasses.fields(PolicyBundle)]
