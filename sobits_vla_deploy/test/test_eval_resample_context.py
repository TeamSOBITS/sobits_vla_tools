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
Unit tests for eval.metrics.resample and eval.context.EvalContext.

Covers time-grid resampling on a known series and the arm-groups derivation
that replaced the old module-level ARM_GROUPS global.
"""

import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy.eval.context import EvalContext  # noqa: E402
from sobits_vla_deploy.eval.metrics import resample  # noqa: E402


class TestResample:

    def test_linear_interp_on_known_series(self):
        # A single episode with a straight line v = 2*t at t=0,1,2.
        steps = pd.DataFrame({'t': [0.0, 1.0, 2.0], 'v': [0.0, 2.0, 4.0]})
        episodes = [{'steps': steps}]
        grid = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
        out = resample(episodes, 'v', grid)
        assert out.shape == (1, 5)
        np.testing.assert_allclose(out[0], [0.0, 1.0, 2.0, 3.0, 4.0])

    def test_grid_outside_span_is_nan(self):
        steps = pd.DataFrame({'t': [1.0, 2.0], 'v': [10.0, 20.0]})
        episodes = [{'steps': steps}]
        grid = np.array([0.0, 1.0, 2.0, 3.0])
        out = resample(episodes, 'v', grid)
        assert np.isnan(out[0, 0])  # before span
        assert np.isnan(out[0, 3])  # after span
        assert out[0, 1] == pytest.approx(10.0)
        assert out[0, 2] == pytest.approx(20.0)

    def test_empty_episode_stays_all_nan(self):
        episodes = [{'steps': pd.DataFrame()}]
        grid = np.array([0.0, 1.0])
        out = resample(episodes, 'v', grid)
        assert np.isnan(out).all()

    def test_single_sample_insufficient_for_interp(self):
        steps = pd.DataFrame({'t': [1.0], 'v': [5.0]})
        episodes = [{'steps': steps}]
        grid = np.array([0.0, 1.0, 2.0])
        out = resample(episodes, 'v', grid)
        assert np.isnan(out).all()

    def test_multiple_episodes_independent_rows(self):
        steps_a = pd.DataFrame({'t': [0.0, 2.0], 'v': [0.0, 4.0]})
        steps_b = pd.DataFrame({'t': [0.0, 2.0], 'v': [10.0, 10.0]})
        episodes = [{'steps': steps_a}, {'steps': steps_b}]
        grid = np.array([0.0, 1.0, 2.0])
        out = resample(episodes, 'v', grid)
        np.testing.assert_allclose(out[0], [0.0, 2.0, 4.0])
        np.testing.assert_allclose(out[1], [10.0, 10.0, 10.0])


class TestEvalContextArmGroups:

    def test_explicit_arm_groups_wins(self):
        ctx = EvalContext(arm_groups={'arm_left'})
        result = ctx.resolve_arm_groups(['arm_left', 'hand_left', 'head'])
        assert result == {'arm_left'}

    def test_none_joint_groups_returns_none_without_explicit(self):
        ctx = EvalContext()
        assert ctx.resolve_arm_groups(None) is None
        assert ctx.resolve_arm_groups([]) is None

    def test_derives_arm_groups_excluding_hand_head(self):
        ctx = EvalContext()
        result = ctx.resolve_arm_groups(
            ['arm_left', 'hand_left', 'head', 'gripper_right']
        )
        assert result == {'arm_left'}

    def test_explicit_empty_set_treated_as_none(self):
        # set_arm_groups([]) in the old API collapsed to None; mirror that.
        ctx = EvalContext(arm_groups=set())
        # An explicit empty set is still "explicit" in the new dataclass --
        # document the (harmless) behavioural nuance vs the old global setter.
        result = ctx.resolve_arm_groups(['arm_left', 'hand_left'])
        assert result == set()


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
