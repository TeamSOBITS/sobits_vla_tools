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
Seam smoke-test suite — the pass/fail gate for the lerobot 0.5.1 -> 0.6.0 port.

Run standalone with:

    python3 -m pytest sobits_vla_common/test/test_lerobot_seam.py -v

Tests are ordered cheapest-first. On the target (post-port) lerobot version,
test_adapter_symbols_resolve alone enumerates every broken import path.
"""

from __future__ import annotations

import importlib.metadata
import os
from pathlib import Path
import re
import sys

import pytest


# Make sobits_vla_common importable when run standalone (not via colcon/ament).
_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parent.parent
for _pkg_python_dir in (
    _REPO_ROOT / 'sobits_vla_common',
    _REPO_ROOT / 'sobits_vla_rosbag_conversion',
):
    _p = str(_pkg_python_dir)
    if _p not in sys.path:
        sys.path.insert(0, _p)


# ---------------------------------------------------------------------------
# 1. Version detection
# ---------------------------------------------------------------------------


def test_version_detected():
    from sobits_vla_common.lerobot_adapter import LEROBOT_VERSION

    installed = importlib.metadata.version('lerobot')
    parsed = tuple(int(x) for x in installed.split('.')[:3] if x.isdigit())
    assert LEROBOT_VERSION == parsed
    assert len(LEROBOT_VERSION) == 3
    assert all(isinstance(p, int) for p in LEROBOT_VERSION)


# ---------------------------------------------------------------------------
# 2. Every adapter symbol resolves
# ---------------------------------------------------------------------------


def test_adapter_symbols_resolve():
    from sobits_vla_common.lerobot_adapter import describe

    result = describe()
    assert result['unresolvable'] == [], (
        f'lerobot seam broken for these symbols on lerobot {result["version"]}: '
        f'{result["unresolvable"]}'
    )


# ---------------------------------------------------------------------------
# 3. Grep gate: no stray `lerobot` imports outside the two seam files
# ---------------------------------------------------------------------------

# lerobot_adapter.py / lerobot_compat.py: the seam itself.
# test_lerobot_seam.py: this file imports lerobot directly in
# test_compat_patches_apply to assert patch targets still exist — that's
# verification of the private-API coupling, not a new call site.
_ALLOWED_SEAM_FILES = {'lerobot_adapter.py', 'lerobot_compat.py', 'test_lerobot_seam.py'}
_IMPORT_RE = re.compile(r'^\s*(from|import)\s+lerobot\b')


def _iter_repo_python_files(repo_root: Path):
    skip_dirs = {
        '.git', 'build', 'install', 'log', '__pycache__',
        'lerobot_v510', 'lerobot_v600',
        # pixi materializes entire environments (incl. lerobot's own
        # sources) inside the repo — not our call sites.
        '.pixi',
    }
    for path in repo_root.rglob('*.py'):
        if any(part in skip_dirs for part in path.parts):
            continue
        yield path


def test_no_stray_lerobot_imports():
    offenders = []
    for path in _iter_repo_python_files(_REPO_ROOT):
        if path.name in _ALLOWED_SEAM_FILES:
            continue
        try:
            text = path.read_text(encoding='utf-8')
        except (UnicodeDecodeError, OSError):
            continue
        for lineno, line in enumerate(text.splitlines(), start=1):
            if _IMPORT_RE.match(line):
                offenders.append(f'{path.relative_to(_REPO_ROOT)}:{lineno}: {line.strip()}')

    assert offenders == [], (
        'Found stray `lerobot` imports outside lerobot_adapter.py/lerobot_compat.py:\n'
        + '\n'.join(offenders)
    )


# ---------------------------------------------------------------------------
# 4. Processor registry key (patch-4 assumption)
# ---------------------------------------------------------------------------


def test_processor_registry_key():
    from sobits_vla_common.lerobot_adapter import IS_V06, ProcessorStepRegistry
    from sobits_vla_common.lerobot_compat import apply_deploy_patches

    expected_native_key = 'relative_actions_processor' if IS_V06 else 'delta_actions_processor'
    assert expected_native_key in ProcessorStepRegistry._registry

    apply_deploy_patches()

    # After the compat patch, both keys must resolve to the same step class.
    assert 'delta_actions_processor' in ProcessorStepRegistry._registry
    assert 'relative_actions_processor' in ProcessorStepRegistry._registry
    assert (
        ProcessorStepRegistry.get('delta_actions_processor')
        is ProcessorStepRegistry.get('relative_actions_processor')
    )

    # The alias must not change the SERIALIZATION name of the step —
    # register() stamps _registry_name on the class, and pipelines we push
    # must keep the native key so stock lerobot can load them.
    step_cls = ProcessorStepRegistry.get(expected_native_key)
    assert getattr(step_cls, '_registry_name', expected_native_key) == expected_native_key


# ---------------------------------------------------------------------------
# 5. Compat patches apply cleanly
# ---------------------------------------------------------------------------


def test_compat_patches_apply():
    from sobits_vla_common import lerobot_compat as lc

    # apply_conversion_patches() is a documented no-op on lerobot 0.6.0 —
    # RunningQuantileStats.update() promotes via np.result_type upstream now
    # (fix #3697), so the old uint8-overflow patch was deleted rather than
    # kept as a dead version gate. It must still be callable so call sites
    # that don't know about that don't need a version check.
    assert callable(lc.apply_conversion_patches)
    lc.apply_conversion_patches()
    lc.apply_training_patches()
    lc.apply_deploy_patches()

    # Idempotency flags set.
    assert lc._bool_quantile_normalization_patched
    assert lc._pi05_action_dim_padding_patched
    assert lc._processor_registry_patched
    assert lc._pi0fast_peft_targets_patched
    assert lc._pi05_from_pretrained_patched
    assert lc._vla_jepa_image_resize_patched

    # The resize helper backing the VLA-JEPA patch must equalize
    # heterogeneous camera resolutions for both frame and video tensors.
    import torch
    batch = {
        'observation.images.a': torch.zeros(2, 3, 480, 640),
        'observation.images.b': torch.zeros(2, 8, 3, 1200, 1920),
        'action': torch.zeros(2, 7, 19),
    }
    resized = lc._resize_image_features(
        batch, ['observation.images.a', 'observation.images.b'], (480, 640)
    )
    assert resized['observation.images.a'].shape == (2, 3, 480, 640)
    assert resized['observation.images.b'].shape == (2, 8, 3, 480, 640)
    assert resized['action'].shape == (2, 7, 19)
    # Untouched tensors are passed through, not copied.
    assert resized['observation.images.a'] is batch['observation.images.a']

    # State-dim introspection must read a per-dimension stat, never scalars
    # like 'count' — that once yielded expected_state_dim=1 and truncated
    # the 19-dim state at deploy time.
    sys.path.insert(0, str(_REPO_ROOT / 'sobits_vla_deploy'))
    from sobits_vla_deploy.policy_loader import _state_dim_from_preprocessor

    class _FakeStep:
        _tensor_stats = {
            'observation.state': {
                'count': torch.ones(1),
                'mean': torch.zeros(19),
                'std': torch.ones(19),
            }
        }

    class _FakePipeline:
        steps = [_FakeStep()]

    assert _state_dim_from_preprocessor(_FakePipeline()) == 19

    # Patch targets import and are (still) patched in place — hard fail if
    # the target class/attr is missing outright, since every patch below is
    # unconditionally active on lerobot 0.6.0 (see lerobot_compat.py).
    from lerobot.datasets.compute_stats import RunningQuantileStats
    assert hasattr(RunningQuantileStats, 'update')

    from lerobot.processor.normalize_processor import _NormalizationMixin
    assert hasattr(_NormalizationMixin, '_apply_transform')

    for module_path, cls_name in (
        ('lerobot.policies.pi0.modeling_pi0', 'PI0Policy'),
        ('lerobot.policies.pi05.modeling_pi05', 'PI05Policy'),
        ('lerobot.policies.pi0_fast.modeling_pi0_fast', 'PI0FastPolicy'),
    ):
        mod = importlib.import_module(module_path)
        cls = getattr(mod, cls_name)
        assert hasattr(cls, '_fix_pytorch_state_dict_keys')


# ---------------------------------------------------------------------------
# 6. policy_registry.make_policy_config for every registry entry
# ---------------------------------------------------------------------------


def test_policy_config_builds():
    from sobits_vla_common.policy_registry import available_policies, make_policy_config

    policies = available_policies()
    assert policies, 'policy_registry has no entries'

    for policy_type in policies:
        try:
            cfg = make_policy_config(policy_type, overrides={}, device='cpu')
        except ModuleNotFoundError as exc:
            pytest.skip(
                f'{policy_type}: lerobot module not present in this install ({exc})'
            )
            continue
        except Exception as exc:
            pytest.fail(f'{policy_type}: make_policy_config raised {type(exc).__name__}: {exc}')
        else:
            assert cfg is not None


# ---------------------------------------------------------------------------
# 7. Dataset create -> add_frame -> save_episode -> finalize -> reload
# ---------------------------------------------------------------------------


def test_dataset_roundtrip(tmp_path):
    import numpy as np

    from sobits_vla_common.lerobot_adapter import LeRobotDataset
    from sobits_vla_rosbag_conversion.dataset_writer import (
        _make_create_kwargs,
        read_custom_info,
        write_custom_info,
    )

    features = {
        'action': {'dtype': 'float32', 'shape': (2,), 'names': ['j0', 'j1']},
        'observation.state': {'dtype': 'float32', 'shape': (2,), 'names': ['j0', 'j1']},
    }
    repo_id = 'sobits_vla_seam_test_dataset'
    root = tmp_path / 'ds'

    # Same kwargs-building path production conversion uses (dataset_writer).
    create_kwargs = _make_create_kwargs(
        dataset_name=repo_id,
        fps=10,
        features=features,
        output_directory=root,
        robot_type='seam_test_robot',
        vcodec='libsvtav1',
    )
    dataset = LeRobotDataset.create(**create_kwargs)

    # lerobot 0.6.0's meta/info.json is a typed DatasetInfo dataclass with no
    # robot_info/user_info fields (see dataset_writer.write_custom_info) — the
    # production writer persists them to a meta/ sidecar JSON file instead of
    # dataset.meta.info[...]. Exercise that exact path here.
    custom_robot_info = {'name': 'seam_test_robot', 'version': '1'}
    custom_user_info = {'lerobot_version': '0.6.0', 'sobits_vla_tools_rev': 'testrev'}
    write_custom_info(dataset.root, robot_info=custom_robot_info, user_info=custom_user_info)

    for i in range(2):
        dataset.add_frame({
            'action': np.array([0.1 * i, 0.2 * i], dtype=np.float32),
            'observation.state': np.array([0.1 * i, 0.2 * i], dtype=np.float32),
            'task': 'seam smoke test',
        })
    dataset.save_episode()
    dataset.finalize()

    reloaded = LeRobotDataset(repo_id, root=root)
    reloaded_custom_info = read_custom_info(reloaded.root)

    assert reloaded.meta.total_episodes == 1
    assert reloaded_custom_info.get('robot_info') == custom_robot_info
    assert reloaded_custom_info.get('user_info') == custom_user_info


# ---------------------------------------------------------------------------
# 8. Full network+GPU smoke test (opt-in only)
# ---------------------------------------------------------------------------


@pytest.mark.skipif(
    not os.environ.get('SOBITS_VLA_FULL_SEAM'),
    reason='Set SOBITS_VLA_FULL_SEAM=1 to run the network-dependent smolvla load test.',
)
def test_from_pretrained_smolvla():
    from sobits_vla_common.lerobot_adapter import make_pre_post_processors
    from sobits_vla_common.policy_registry import get_entry

    entry = get_entry('smolvla')
    module_path, class_name = entry.class_path.rsplit('.', 1)
    policy_module = importlib.import_module(module_path)
    policy_cls = getattr(policy_module, class_name)

    repo_id = entry.default_pretrained
    assert repo_id, 'smolvla registry entry has no default_pretrained repo_id'

    policy = policy_cls.from_pretrained(repo_id)
    assert policy is not None

    preprocessor, postprocessor = make_pre_post_processors(policy.config, repo_id)
    assert preprocessor is not None
    assert postprocessor is not None
