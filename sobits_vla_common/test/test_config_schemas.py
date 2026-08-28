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
Sweep every shipped config YAML against its node's _SCHEMA.

Catches the dead-key class (a config key no node ever declares -- ROS ignores
it silently) at test time. Schemas are extracted from the node source with ast
so this test needs neither rclpy nor the nodes' heavy imports.
"""

import ast
import pathlib
import re
import sys

import pytest

_TEST_DIR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_TEST_DIR.parent))

from sobits_vla_common.param_schema import P, Template, validate_config  # noqa: E402

_REPO = _TEST_DIR.parent.parent


def _load_schema(rel_py: str):
    """Exec only the module's `_SCHEMA = {...}` literal; descriptors stubbed to None."""
    src = (_REPO / rel_py).read_text()
    for node in ast.walk(ast.parse(src)):
        if (isinstance(node, ast.Assign)
                and any(getattr(t, 'id', '') == '_SCHEMA' for t in node.targets)):
            mod = ast.Module(body=[node], type_ignores=[])
            ns = {'P': P, 'Template': Template, '_pd': lambda desc: None}
            exec(compile(mod, rel_py, 'exec'), ns)
            return ns['_SCHEMA']
    raise AssertionError('no _SCHEMA found in {}'.format(rel_py))


# (schema source, node name, config glob, allowed-unknown regexes).
# Allowed = keys those nodes declare dynamically at runtime, invisible to the
# static schema; anything else unknown is a typo or a dead key.
_CASES = [
    (
        'sobits_vla_common/sobits_vla_common/world_reset_node.py',
        'world_reset_node',
        'sobits_vla_common/config/world_reset*.yaml',
        # Per-preset scene blocks and per-group reset_pose blocks.
        [r'^world_reset\.reset_pose\.[^.]+\..+', r'^world_reset\.[^.]+\..+'],
    ),
    (
        'sobits_vla_deploy/sobits_vla_deploy/sobits_vla_deploy.py',
        'sobits_vla_deploy',
        'sobits_vla_deploy/config/deploy_config*.yaml',
        # robot.* is declared in _load_robot_profile from the descriptor.
        [r'^robot\..+'],
    ),
    (
        'sobits_vla_training/sobits_vla_training/train_node.py',
        'sobits_vla_training',
        'sobits_vla_training/config/training_config*.yaml',
        [r'^peft\.full_training_modules$', r'^policy_overrides($|\..+)'],
    ),
    (
        'sobits_vla_rosbag_conversion/sobits_vla_rosbag_conversion/'
        'ros2bag_to_lerobotdataset.py',
        'rosbag_conversion_node',
        'sobits_vla_rosbag_conversion/config/conversion_config*.yaml',
        [],
    ),
]


def _params():
    for schema_py, node_name, glob, allowed in _CASES:
        for cfg in sorted(_REPO.glob(glob)):
            yield pytest.param(
                schema_py, node_name, cfg, allowed,
                id='{}:{}'.format(node_name, cfg.name),
            )


@pytest.mark.parametrize('schema_py, node_name, cfg, allowed', list(_params()))
def test_config_matches_schema(schema_py, node_name, cfg, allowed):
    schema = _load_schema(schema_py)
    problems = validate_config(schema, str(cfg), node_name)
    real = [
        p for p in problems
        if not any(re.search(rx, _problem_key(p)) for rx in allowed)
    ]
    assert not real, 'Unknown config keys (dead or typo):\n' + '\n'.join(real)


def _problem_key(problem: str) -> str:
    """validate_config messages quote the offending dotted key first."""
    m = re.search(r"'([^']+)'", problem)
    return m.group(1) if m else problem


def test_every_case_found_configs():
    for _, node_name, glob, _ in _CASES:
        assert list(_REPO.glob(glob)), 'no configs matched {}'.format(glob)
