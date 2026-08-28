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
Declarative ROS parameter schemas: one nested dict drives declare + read + validate.

No rclpy import -- unit-testable without a ROS graph. A schema is a nested dict
whose leaves are P(default, description); nesting builds dotted names, e.g.
{'runtime': {'control_hz': P(10.0)}} -> 'runtime.control_hz'. Template marks a
subtree that repeats once per item of a list-valued key parameter declared
earlier in the same schema (see robot_info.morphology.<part>.* in
sobits_vla_rosbag_collection/src/rosbag_collection.cpp:85-110 for the pattern
this mirrors).

Node objects are duck-typed (declare_parameter/get_parameter only), so tests
pass a fake node instead of constructing rclpy.node.Node.
"""

from __future__ import annotations

from dataclasses import dataclass
import types
from typing import Any, Dict, List, Union


@dataclass
class P:
    default: Any
    description: str = ''


@dataclass
class Template:
    """Subtree instantiated once per item of the list param at key_param."""

    key_param: str
    subtree: Dict[str, Any]


SchemaNode = Union[P, Template, Dict[str, Any]]


def _join(ns: str, name: str) -> str:
    return '{}.{}'.format(ns, name) if ns else name


def _is_str_list_default(default: Any) -> bool:
    return isinstance(default, list) and all(isinstance(v, str) for v in default)


def _declare_leaf(node: Any, name: str, leaf: P) -> None:
    node.declare_parameter(name, leaf.default)


def declare_from_schema(node: Any, schema: Dict[str, Any], ns: str = '') -> None:
    """
    Walk the schema, declaring every leaf.

    A Template's key_param must already be declared (either earlier in this
    same schema, at a lower nesting depth, or by the caller) so its item list
    can be read before the subtree expands.
    """
    for key, value in schema.items():
        name = _join(ns, key)
        if isinstance(value, P):
            _declare_leaf(node, name, value)
        elif isinstance(value, Template):
            _declare_template(node, name, value)
        elif isinstance(value, dict):
            declare_from_schema(node, value, ns=name)
        else:
            raise TypeError('Unsupported schema node at {!r}: {!r}'.format(name, value))


def _declare_template(node: Any, ns: str, tmpl: Template) -> None:
    items = node.get_parameter(tmpl.key_param).value or []
    for item in items:
        declare_from_schema(node, tmpl.subtree, ns=_expand_template_ns(ns, item))


def _expand_template_ns(ns: str, item: str) -> str:
    return ns.replace('<item>', str(item))


def read_schema(node: Any, schema: Dict[str, Any], ns: str = '') -> types.SimpleNamespace:
    """Mirror schema as a nested SimpleNamespace, values from node.get_parameter."""
    out = types.SimpleNamespace()
    for key, value in schema.items():
        name = _join(ns, key)
        if isinstance(value, P):
            setattr(out, key, _read_leaf(node, name, value))
        elif isinstance(value, Template):
            # '<item>' is a path placeholder, not a real attribute: its
            # per-item results land directly on `out`, keyed by item name.
            for item_key, item_ns in _read_template(node, name, value):
                setattr(out, item_key, item_ns)
        elif isinstance(value, dict):
            setattr(out, key, read_schema(node, value, ns=name))
        else:
            raise TypeError('Unsupported schema node at {!r}: {!r}'.format(name, value))
    return out


def _read_leaf(node: Any, name: str, leaf: P) -> Any:
    value = node.get_parameter(name).value
    # Empty-string sentinel: str[] params can't be declared with no default
    # type, so [''] stands for "empty list" and is filtered back out on read.
    if _is_str_list_default(leaf.default) and isinstance(value, list):
        return [v for v in value if v]
    return value


def _read_template(node: Any, ns: str, tmpl: Template) -> List[tuple]:
    """[(item_name, its subtree namespace), ...] for every item in key_param's list."""
    items = node.get_parameter(tmpl.key_param).value or []
    return [
        (str(item), read_schema(node, tmpl.subtree, ns=_expand_template_ns(ns, item)))
        for item in items
    ]


def _flatten_schema_keys(schema: Dict[str, Any], ns: str = '') -> Dict[str, Any]:
    """Dotted schema key -> leaf/Template, with '<item>' left as a wildcard segment."""
    flat: Dict[str, Any] = {}
    for key, value in schema.items():
        name = _join(ns, key)
        if isinstance(value, P):
            flat[name] = value
        elif isinstance(value, Template):
            flat[name] = value
            flat.update(_flatten_schema_keys(value.subtree, ns=name))
        elif isinstance(value, dict):
            flat.update(_flatten_schema_keys(value, ns=name))
    return flat


def _flatten_yaml_keys(data: Dict[str, Any], ns: str = '') -> List[str]:
    """Every dotted leaf path under a ros__parameters mapping."""
    flat: List[str] = []
    for key, value in data.items():
        name = _join(ns, key)
        if isinstance(value, dict):
            flat.extend(_flatten_yaml_keys(value, ns=name))
        else:
            flat.append(name)
    return flat


def _matches_schema_key(yaml_key: str, schema_keys: List[str]) -> bool:
    """Match yaml_key against a flattened schema key, '<item>' matching any segment."""
    yaml_parts = yaml_key.split('.')
    for schema_key in schema_keys:
        schema_parts = schema_key.split('.')
        if len(schema_parts) != len(yaml_parts):
            continue
        if all(s == '<item>' or s == y for s, y in zip(schema_parts, yaml_parts)):
            return True
    return False


def validate_config(schema: Dict[str, Any], yaml_path: str, node_name: str) -> List[str]:
    """
    Report dotted keys in yaml_path absent from schema.

    Reads node_name's block (or '/**') and flattens it. Missing keys are not
    reported: defaults cover those, only unknown ones indicate a typo.
    """
    import yaml as pyyaml

    with open(yaml_path, 'r') as f:
        doc = pyyaml.safe_load(f) or {}

    node_block = doc.get(node_name)
    if node_block is None:
        node_block = doc.get('/**', {})
    params = (node_block or {}).get('ros__parameters', {})

    schema_keys = list(_flatten_schema_keys(schema).keys())
    problems = []
    for yaml_key in _flatten_yaml_keys(params):
        if not _matches_schema_key(yaml_key, schema_keys):
            problems.append(
                'Unknown parameter {!r} in {} (node {!r}) -- not in schema.'.format(
                    yaml_key, yaml_path, node_name
                )
            )
    return problems
