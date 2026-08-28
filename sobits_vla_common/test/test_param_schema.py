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

"""Unit tests for param_schema -- no rclpy, no ROS graph required."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402

from sobits_vla_common.param_schema import (  # noqa: E402
    declare_from_schema, P, read_schema, Template, validate_config,
)

import yaml  # noqa: E402


class FakeParam:

    def __init__(self, value):
        self.value = value


class FakeNode:
    """Dict-backed stand-in for rclpy.node.Node: declare_parameter/get_parameter only."""

    def __init__(self):
        self._params = {}

    def declare_parameter(self, name, default, descriptor=None):
        self._params[name] = default
        return FakeParam(default)

    def get_parameter(self, name):
        return FakeParam(self._params[name])

    def set_value(self, name, value):
        self._params[name] = value


def _morphology_schema():
    return {
        'robot_info': {
            'name': P('sobit_robot'),
            'morphology': {
                'parts': P(['base', 'arm']),
                '<item>': Template('robot_info.morphology.parts', {
                    'is_actionable': P(False),
                    'joint_names': P(['']),
                }),
            },
        },
    }


class TestDeclareFromSchema:

    def test_nested_dotted_names(self):
        node = FakeNode()
        schema = {'runtime': {'control_hz': P(10.0, 'loop rate')}}
        declare_from_schema(node, schema)
        assert node._params['runtime.control_hz'] == 10.0

    def test_namespace_prefix(self):
        node = FakeNode()
        schema = {'a': {'b': P(1)}}
        declare_from_schema(node, schema, ns='world_reset')
        assert node._params['world_reset.a.b'] == 1

    def test_multiple_leaves_at_different_depths(self):
        node = FakeNode()
        schema = {
            'x': P(1.0),
            'group': {'y': P('hi'), 'z': P([1, 2, 3])},
        }
        declare_from_schema(node, schema)
        assert node._params == {'x': 1.0, 'group.y': 'hi', 'group.z': [1, 2, 3]}


class TestReadSchema:

    def test_round_trip_with_namespace(self):
        node = FakeNode()
        schema = {'runtime': {'control_hz': P(10.0)}}
        declare_from_schema(node, schema)
        node.set_value('runtime.control_hz', 20.0)
        ns = read_schema(node, schema)
        assert ns.runtime.control_hz == 20.0

    def test_returns_simple_namespace_shape(self):
        node = FakeNode()
        schema = {'a': {'b': P(1), 'c': P('s')}}
        declare_from_schema(node, schema)
        ns = read_schema(node, schema)
        assert ns.a.b == 1
        assert ns.a.c == 's'

    def test_empty_string_list_sentinel_filtered(self):
        node = FakeNode()
        schema = {'names': P([''])}
        declare_from_schema(node, schema)
        ns = read_schema(node, schema)
        assert ns.names == []

    def test_non_sentinel_str_list_passes_through(self):
        node = FakeNode()
        schema = {'names': P([''])}
        declare_from_schema(node, schema)
        node.set_value('names', ['a', '', 'b'])
        ns = read_schema(node, schema)
        assert ns.names == ['a', 'b']

    def test_non_str_list_default_not_filtered(self):
        node = FakeNode()
        schema = {'vals': P([1, 2])}
        declare_from_schema(node, schema)
        node.set_value('vals', [1, 2, 3])
        ns = read_schema(node, schema)
        assert ns.vals == [1, 2, 3]


class TestTemplate:

    def test_declare_expands_per_item(self):
        node = FakeNode()
        schema = _morphology_schema()
        declare_from_schema(node, schema)
        assert 'robot_info.morphology.base.is_actionable' in node._params
        assert 'robot_info.morphology.arm.is_actionable' in node._params
        assert node._params['robot_info.morphology.base.is_actionable'] is False

    def test_read_expands_per_item(self):
        node = FakeNode()
        schema = _morphology_schema()
        declare_from_schema(node, schema)
        node.set_value('robot_info.morphology.arm.is_actionable', True)
        ns = read_schema(node, schema)
        assert ns.robot_info.morphology.base.is_actionable is False
        assert ns.robot_info.morphology.arm.is_actionable is True

    def test_empty_key_list_expands_nothing(self):
        node = FakeNode()
        schema = _morphology_schema()
        node.declare_parameter('robot_info.morphology.parts', [])
        node.declare_parameter('robot_info.name', 'sobit_robot')
        declare_from_schema(node, {
            'robot_info': {
                'morphology': {
                    '<item>': schema['robot_info']['morphology']['<item>'],
                },
            },
        })
        assert node._params == {
            'robot_info.morphology.parts': [],
            'robot_info.name': 'sobit_robot',
        }


class TestValidateConfig:

    def _write(self, tmp_path, doc):
        path = tmp_path / 'config.yaml'
        path.write_text(yaml.safe_dump(doc))
        return str(path)

    def test_valid_file_has_no_problems(self, tmp_path):
        schema = {'runtime': {'control_hz': P(10.0)}}
        path = self._write(tmp_path, {
            'my_node': {'ros__parameters': {'runtime': {'control_hz': 20.0}}},
        })
        assert validate_config(schema, path, 'my_node') == []

    def test_typo_key_is_reported(self, tmp_path):
        schema = {'runtime': {'control_hz': P(10.0)}}
        path = self._write(tmp_path, {
            'my_node': {'ros__parameters': {'runtime': {'contorl_hz': 20.0}}},
        })
        problems = validate_config(schema, path, 'my_node')
        assert len(problems) == 1
        assert 'runtime.contorl_hz' in problems[0]

    def test_missing_key_is_not_a_problem(self, tmp_path):
        schema = {'runtime': {'control_hz': P(10.0), 'other': P(1)}}
        path = self._write(tmp_path, {
            'my_node': {'ros__parameters': {'runtime': {'control_hz': 20.0}}},
        })
        assert validate_config(schema, path, 'my_node') == []

    def test_wildcard_node_key_accepted(self, tmp_path):
        schema = {'runtime': {'control_hz': P(10.0)}}
        path = self._write(tmp_path, {
            '/**': {'ros__parameters': {'runtime': {'control_hz': 20.0}}},
        })
        assert validate_config(schema, path, 'any_node_name') == []

    def test_template_section_valid_for_any_item_name(self, tmp_path):
        schema = _morphology_schema()
        path = self._write(tmp_path, {
            '/**': {'ros__parameters': {
                'robot_info': {
                    'name': 'sobit_robot',
                    'morphology': {
                        'parts': ['base', 'arm', 'gripper'],
                        'base': {'is_actionable': False, 'joint_names': []},
                        'gripper': {'is_actionable': True, 'joint_names': ['g1']},
                    },
                },
            }},
        })
        assert validate_config(schema, path, 'n') == []

    def test_template_section_catches_typo_inside_item(self, tmp_path):
        schema = _morphology_schema()
        path = self._write(tmp_path, {
            '/**': {'ros__parameters': {
                'robot_info': {
                    'name': 'sobit_robot',
                    'morphology': {
                        'parts': ['base'],
                        'base': {'is_actionalbe': False},
                    },
                },
            }},
        })
        problems = validate_config(schema, path, 'n')
        assert len(problems) == 1
        assert 'robot_info.morphology.base.is_actionalbe' in problems[0]


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
