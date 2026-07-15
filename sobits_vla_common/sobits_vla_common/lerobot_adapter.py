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
Single import seam for every lerobot symbol the bridge uses.

All other sobits_vla_tools modules import lerobot symbols from here (via
``from sobits_vla_common.lerobot_adapter import X``) instead of reaching into
lerobot directly. This keeps the lerobot import surface centralized in
exactly two files: this module and ``lerobot_compat.py``.

sobits_vla_tools targets lerobot >= 0.6.0 only; 0.5.1 support was dropped
(see docs/lerobot_v060_migration_plan.md). The last 0.5.1-compatible
revision is on branch ``feat/refactor``.

Resolution is lazy (PEP 562 module ``__getattr__``) so light consumers that
only need e.g. ``HF_LEROBOT_HOME`` don't pay the torch/lerobot import cost at
ROS node startup.
"""

from __future__ import annotations

import importlib
from importlib.metadata import version as _pkg_version


def _parse(v: str) -> tuple:
    """Parse a PEP 440-ish version string into a bare (major, minor, patch) tuple."""
    return tuple(int(x) for x in v.split('.')[:3] if x.isdigit())


LEROBOT_VERSION: tuple = _parse(_pkg_version('lerobot'))
IS_V06: bool = LEROBOT_VERSION >= (0, 6)

if LEROBOT_VERSION < (0, 6):
    _found = '.'.join(str(p) for p in LEROBOT_VERSION)
    raise ImportError(
        f'sobits_vla_tools now requires lerobot >= 0.6.0; found {_found}. '
        'The last 0.5.1-compatible revision is on branch feat/refactor.'
    )


# name -> backing module. lerobot.datasets.feature_utils.{build_dataset_frame,
# hw_to_dataset_features} moved to lerobot.utils.feature_utils, and
# lerobot.utils.control_utils.predict_action moved to
# lerobot.common.control_utils in 0.6.0 — see docs/lerobot_v060_changes.md §7.
_SYMBOLS: dict[str, str] = {
    'LeRobotDataset':                    'lerobot.datasets.lerobot_dataset',
    'LeRobotDatasetMetadata':            'lerobot.datasets.lerobot_dataset',
    'HF_LEROBOT_HOME':                   'lerobot.utils.constants',
    'RunningQuantileStats':              'lerobot.datasets.compute_stats',
    'build_dataset_frame':               'lerobot.utils.feature_utils',
    'hw_to_dataset_features':            'lerobot.utils.feature_utils',
    'DatasetConfig':                     'lerobot.configs.default',
    'WandBConfig':                       'lerobot.configs.default',
    'PeftConfig':                        'lerobot.configs.default',
    'TrainPipelineConfig':               'lerobot.configs.train',
    'FeatureType':                       'lerobot.configs.types',
    'PolicyFeature':                     'lerobot.configs.types',
    'RTCAttentionSchedule':              'lerobot.configs.types',
    'PreTrainedConfig':                  'lerobot.configs.policies',
    'make_policy':                       'lerobot.policies',
    'make_pre_post_processors':          'lerobot.policies',
    'prepare_observation_for_inference': 'lerobot.policies.utils',
    'predict_action':                    'lerobot.common.control_utils',
    'LatencyTracker':                    'lerobot.policies.rtc.latency_tracker',
    'RTCConfig':                         'lerobot.policies.rtc.configuration_rtc',
    'ProcessorStepRegistry':             'lerobot.processor.pipeline',
    'AbsoluteActionsProcessorStep':      'lerobot.processor.relative_action_processor',
    'RGBEncoderConfig':                  'lerobot.configs.video',
    'train':                             'lerobot.scripts.lerobot_train',
}


def __getattr__(name: str):
    """PEP 562 lazy module attribute resolution — imports the backing module on first use."""
    try:
        mod_name = _SYMBOLS[name]
    except KeyError:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}') from None
    module = importlib.import_module(mod_name)
    return getattr(module, name)


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(_SYMBOLS))


def describe() -> dict:
    """
    Resolve every symbol in the seam and report what's broken.

    Returns
    -------
    dict
        ``{'version': (0, 6, 0), 'is_v06': True, 'unresolvable': [...]}``
        where ``unresolvable`` lists ``(symbol, error)`` pairs for every
        symbol that failed to import in the active lerobot version. Meant to
        be logged once at node startup and used by the seam test suite.

    """
    unresolvable: list[tuple[str, str]] = []
    for name in _SYMBOLS:
        try:
            __getattr__(name)
        except Exception as exc:  # noqa: BLE001 - collecting all failures, not just the first
            unresolvable.append((name, f'{type(exc).__name__}: {exc}'))
    return {
        'version': LEROBOT_VERSION,
        'is_v06': IS_V06,
        'unresolvable': unresolvable,
    }
