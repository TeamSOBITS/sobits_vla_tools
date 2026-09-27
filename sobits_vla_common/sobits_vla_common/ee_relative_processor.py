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
LeRobot processor steps for SE(3) EE relative actions (UMI-style).

Mirror of lerobot's RelativeActionsProcessorStep / AbsoluteActionsProcessorStep
for `ee.<arm>.*` groups: the preprocessor caches observation.state and maps
actions to A_k = inv(T_obs) . T_k; the postprocessor composes T_obs . A_k.
Registered names travel in the checkpoint's processor JSON, so deploy picks
the pair up through the normal pipeline load + reconnect_ee_relative_steps.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence, Tuple

from sobits_vla_common.ee_relative import (
    ee_groups_from_names, EEGroup, to_absolute_ee, to_relative_ee,
)
from sobits_vla_common.lerobot_adapter import (
    NormalizerProcessorStep, OBS_STATE, PipelineFeatureType, PolicyFeature, ProcessorStep,
    ProcessorStepRegistry, TransitionKey, UnnormalizerProcessorStep,
)
import torch

EE_RELATIVE_STEP_NAME = 'sobits_ee_relative_actions'
EE_ABSOLUTE_STEP_NAME = 'sobits_ee_absolute_actions'


@ProcessorStepRegistry.register(EE_RELATIVE_STEP_NAME)
@dataclass
class EERelativeActionsProcessorStep(ProcessorStep):
    """
    Convert absolute EE actions to observation-relative SE(3) poses.

    Always caches observation.state for the paired EEAbsoluteActionsProcessorStep,
    even when disabled or when the transition carries no action (inference).
    """

    enabled: bool = False
    action_names: List[str] = field(default_factory=list)
    state_names: List[str] = field(default_factory=list)
    _last_state: Optional[torch.Tensor] = field(default=None, init=False, repr=False)
    _groups: List[EEGroup] = field(default_factory=list, init=False, repr=False)

    def __post_init__(self):
        self.action_names = list(self.action_names)
        self.state_names = list(self.state_names)
        self._groups = ee_groups_from_names(self.action_names, self.state_names)

    @property
    def groups(self) -> List[EEGroup]:
        return self._groups

    def __call__(self, transition):
        observation = transition.get(TransitionKey.OBSERVATION, {})
        state = observation.get(OBS_STATE) if observation else None
        if state is not None:
            self._last_state = state

        if not self.enabled:
            return transition

        new_transition = transition.copy()
        action = new_transition.get(TransitionKey.ACTION)
        if action is None or state is None:
            return new_transition
        new_transition[TransitionKey.ACTION] = to_relative_ee(action, state, self._groups)
        return new_transition

    def get_cached_state(self) -> Optional[torch.Tensor]:
        """Observation state the next postprocessor call composes onto (UMI's T_obs_latest)."""
        return self._last_state

    def get_config(self) -> dict[str, Any]:
        return {
            'enabled': self.enabled,
            'action_names': list(self.action_names),
            'state_names': list(self.state_names),
        }

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features


@ProcessorStepRegistry.register(EE_ABSOLUTE_STEP_NAME)
@dataclass
class EEAbsoluteActionsProcessorStep(ProcessorStep):
    """Compose relative EE actions back onto the paired step's cached observation pose."""

    enabled: bool = False
    relative_step: Optional[EERelativeActionsProcessorStep] = field(default=None, repr=False)

    def __call__(self, transition):
        if not self.enabled:
            return transition

        if self.relative_step is None:
            raise RuntimeError(
                'EEAbsoluteActionsProcessorStep has no paired EERelativeActionsProcessorStep; '
                'call reconnect_ee_relative_steps(preprocessor, postprocessor) after loading.')
        cached_state = self.relative_step.get_cached_state()
        if cached_state is None:
            raise RuntimeError(
                'EEAbsoluteActionsProcessorStep needs the observation state cached by '
                'EERelativeActionsProcessorStep; run the preprocessor before the postprocessor.')

        new_transition = transition.copy()
        action = new_transition.get(TransitionKey.ACTION)
        if action is None:
            return new_transition
        new_transition[TransitionKey.ACTION] = to_absolute_ee(
            action, cached_state, self.relative_step.groups)
        return new_transition

    def get_config(self) -> dict[str, Any]:
        return {'enabled': self.enabled}

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features


def insert_ee_relative_steps(
    preprocessor, postprocessor, action_names: Sequence[str], state_names: Sequence[str],
) -> Tuple[EERelativeActionsProcessorStep, EEAbsoluteActionsProcessorStep]:
    """
    Add the EE relative/absolute pair around (un)normalization; idempotent.

    Relative goes right before the first NormalizerProcessorStep, absolute right
    after the first UnnormalizerProcessorStep. Any existing pair is replaced.
    """
    relative = EERelativeActionsProcessorStep(
        enabled=True, action_names=list(action_names), state_names=list(state_names))
    if not relative.groups:
        raise ValueError('insert_ee_relative_steps: no ee.<arm>.<axis> action names')

    pre_steps = [s for s in preprocessor.steps
                 if not isinstance(s, EERelativeActionsProcessorStep)]
    post_steps = [s for s in postprocessor.steps
                  if not isinstance(s, EEAbsoluteActionsProcessorStep)]
    norm_i = next(
        (i for i, s in enumerate(pre_steps) if isinstance(s, NormalizerProcessorStep)), None)
    unnorm_i = next(
        (i for i, s in enumerate(post_steps) if isinstance(s, UnnormalizerProcessorStep)), None)
    if norm_i is None:
        raise ValueError('preprocessor has no NormalizerProcessorStep to insert before')
    if unnorm_i is None:
        raise ValueError('postprocessor has no UnnormalizerProcessorStep to insert after')

    absolute = EEAbsoluteActionsProcessorStep(enabled=True, relative_step=relative)
    pre_steps.insert(norm_i, relative)
    post_steps.insert(unnorm_i + 1, absolute)
    preprocessor.steps = pre_steps
    postprocessor.steps = post_steps
    return relative, absolute


def reconnect_ee_relative_steps(preprocessor, postprocessor) -> None:
    """Re-link EEAbsoluteActionsProcessorStep.relative_step after pipelines are deserialized."""
    relative = next(
        (s for s in preprocessor.steps if isinstance(s, EERelativeActionsProcessorStep)), None)
    if relative is None:
        return
    for step in postprocessor.steps:
        if isinstance(step, EEAbsoluteActionsProcessorStep) and step.relative_step is None:
            step.relative_step = relative
