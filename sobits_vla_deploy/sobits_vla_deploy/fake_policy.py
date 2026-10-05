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

"""Torch-free stand-in policy for the `model.fake_policy` dry run (no rclpy)."""

from dataclasses import dataclass
import time
from typing import Any, Dict, List, Optional, Tuple

from sobits_vla_common.robot_descriptor import ee_rotation_from_names


class FakePolicy:
    """Holds the measured pose: velocity keys get zero, everything else its measured value."""

    def __init__(self, action_keys: List[str], actions_per_chunk: int,
                 latency_s: float = 0.05) -> None:
        self.action_keys = list(action_keys)
        self.actions_per_chunk = max(int(actions_per_chunk), 1)
        self.latency_s = latency_s

    def predict(
        self, obs_frame: Dict[str, Any], state_vector: Dict[str, float],
    ) -> Tuple[List[Dict[str, float]], None, int]:
        """Return (steps, raw_model_chunk, used_delay) like InferenceEngine._predict_actions."""
        time.sleep(self.latency_s)
        step = {
            key: 0.0 if key.endswith('.vel') else float(state_vector.get(key, 0.0))
            for key in self.action_keys
        }
        return [dict(step) for _ in range(self.actions_per_chunk)], None, 0


@dataclass
class FakeBundle:
    """Same fields as policy_loader.PolicyBundle, without importing it (it needs torch)."""

    policy: Any
    rtc_enabled: bool
    model_action_feature_names: Optional[List[str]]
    model_use_relative_actions: bool
    model_ee_relative: bool
    model_ee_rotation: str
    expected_state_dim: Optional[int]
    preprocessor: Any
    postprocessor: Any


def fake_bundle(action_keys: List[str], actions_per_chunk: int) -> FakeBundle:
    """Build the bundle the node applies in place of PolicyLoader.load_policy()."""
    return FakeBundle(
        policy=FakePolicy(action_keys, actions_per_chunk),
        rtc_enabled=False,
        model_action_feature_names=list(action_keys),
        model_use_relative_actions=False,
        model_ee_relative=False,
        model_ee_rotation=ee_rotation_from_names(list(action_keys)),
        expected_state_dim=None,
        preprocessor=None,
        postprocessor=None,
    )
