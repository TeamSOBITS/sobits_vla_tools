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
Command-line scaffolding utility for generating new robot descriptor configs.

Enables quick onboarding of new robot platforms by generating yaml stubs.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

from sobits_vla_common.robot_descriptor import (
    _parse_descriptor_file,
    load_robot_descriptor,
    resolve_descriptor_path,
    validate_descriptor,
)


def generate_descriptor_yaml(
    robot_id: str, dof: int, cameras: list[str], mobile_base: str
) -> str:
    """Generate structured YAML content for a robot descriptor scaffold."""
    yaml_lines = [
        'schema_version: 1',
        f'robot_id: {robot_id}',
        '',
        f'joint_states_topic: /{robot_id}/joint_states',
        '',
        '# ── Joint groups ──────────────────────────────────────────────',
        '# Canonical order: every pipeline stage uses this order for action/state vectors.',
        '# active: false → group excluded from features by default; override per invocation.',
        'groups:',
        '  - name: head',
        f'    command_topic: /{robot_id}/head_position_controller/joint_trajectory',
        f'    command_action: /{robot_id}/head_position_controller/follow_joint_trajectory',
        '    max_joint_delta: 0.10',
        '    active: true',
        '    joints:',
        '      - ros_name: head_pan_joint  # TODO: Verify name',
        '        feature: head_pan_joint',
        '      - ros_name: head_tilt_joint  # TODO: Verify name',
        '        feature: head_tilt_joint',
        '',
        '  - name: arm',
        f'    command_topic: /{robot_id}/arm_position_controller/joint_trajectory',
        f'    command_action: /{robot_id}/arm_position_controller/follow_joint_trajectory',
        '    max_joint_delta: 0.15',
        '    active: true',
        '    joints:',
    ]

    for i in range(1, dof + 1):
        yaml_lines.append(f'      - ros_name: arm_joint{i}  # TODO: Verify name')
        yaml_lines.append(f'        feature: arm_joint{i}')

    if mobile_base != 'none':
        yaml_lines.extend([
            '',
            '# ── Mobile base ────────────────────────────────────────────────',
            'mobile_base:',
            f'  command_topic: /{robot_id}/cmd_vel',
            f'  odom_topic: /{robot_id}/odom',
        ])
        if mobile_base == 'omni':
            yaml_lines.extend([
                '  has_vel_x: true',
                '  has_vel_y: true',
                '  has_vel_z: false',
                '  has_vel_theta: true',
                '  max_vel_x: 0.0',
                '  max_vel_y: 0.0',
                '  max_vel_z: 0.0',
                '  max_vel_theta: 0.0',
                '  features: [x.vel, y.vel, theta.vel]',
            ])
        else:  # diff
            yaml_lines.extend([
                '  has_vel_x: true',
                '  has_vel_y: false',
                '  has_vel_z: false',
                '  has_vel_theta: true',
                '  max_vel_x: 0.0',
                '  max_vel_y: 0.0',
                '  max_vel_z: 0.0',
                '  max_vel_theta: 0.0',
                '  features: [x.vel, theta.vel]',
            ])
    else:
        yaml_lines.extend([
            '',
            'mobile_base: null',
        ])

    yaml_lines.extend([
        '',
        '# ── Sensors ───────────────────────────────────────────────────',
        'sensors:',
        '  cameras:',
    ])

    for cam in cameras:
        yaml_lines.extend([
            f'    - name: {cam}_camera',
            f'      compressed_topic: /{robot_id}/{cam}_camera/color/image_raw/compressed',
            f'      raw_topic: /{robot_id}/{cam}_camera/color/image_raw',
            f'      info_topic: /{robot_id}/{cam}_camera/color/camera_info',
            '      encoding: ""',
            '      compressed: true',
            '      active: true',
        ])

    yaml_lines.extend([
        '',
        '# ── End-effector TF poses ─────────────────────────────────────',
        'ee_poses:',
        '  - name: left  # TODO: Verify name',
        '    source_frame: hand_left_end_effector_link  # TODO: Verify source frame',
        '    target_frame: base_footprint  # TODO: Verify target frame',
        '',
        '# ── Excluded joints ──────────────────────────────────────────',
        'excluded_joints:',
        '  # - mimic_joint_placeholder  # TODO: Verify mimic / excluded joints',
    ])

    return '\n'.join(yaml_lines) + '\n'


def generate_collection_config_yaml(robot_id: str) -> str:
    """Generate YAML content for a collection config stub matching the descriptor."""
    lines = [
        '/**:',
        '  ros__parameters:',
        '    # Morphology (joint groups, command topics, sensors, mobile_base) is loaded',
        f'    # from sobits_vla_common/robots/{robot_id}.robot.yaml via robot_descriptor_id.',
        '    # This is the single source of truth shared across collection, conversion,',
        '    # training, and deploy. Do NOT duplicate joint/topic lists here.',
        f'    robot_descriptor_id: "{robot_id}"',
        '',
        '    user_info:',
        '      name: ""',
        '      location: ""',
        '      email: ""',
        '',
        '    rosbag_config:',
        '      record_directory: ""',
        '      min_episode_duration: 5.0',
        '      max_episode_duration: 0.0',
        '      timestamp_jump_threshold: 1.0',
        '      min_disk_space_mb: 2048',
        '      expected_sensor_fps: 5',
        '      additional_topics:',
        f'        - /{robot_id}/joy',
        '        - /tf',
        '        - /tf_static',
        '      additional_services:',
        '        - ""',
        '      additional_actions:',
        '        - ""',
        '      conversion_format: "mcap"',
        '      compression_mode: "none"',
        '      compression_format: "zstd"',
        '      rmw_serialization_format: "cdr"',
    ]
    return '\n'.join(lines) + '\n'


def check_placeholders(desc_path: Path) -> list[str]:
    """
    Return warnings for unresolved scaffold placeholders in a descriptor file.

    Scans the raw YAML text for ``# TODO`` markers and known placeholder
    tokens emitted by generate_descriptor_yaml (e.g. ``arm_joint1``). These
    pass structural validation but must be edited before real use.
    """
    warnings = []
    try:
        text = desc_path.read_text(encoding='utf-8')
    except Exception as e:
        return [f'Could not read {desc_path}: {e}']

    for i, line in enumerate(text.splitlines(), start=1):
        if '# TODO' in line:
            warnings.append(f'line {i}: unresolved TODO — {line.strip()}')

    # Placeholder joint names the scaffold emits for arm DOF.
    if re.search(r'\barm_joint\d+\b', text):
        warnings.append(
            'placeholder joint name(s) arm_joint<N> still present — '
            'rename to real joint names.'
        )

    return warnings


def main() -> None:
    """Execute new_robot CLI scaffold utility script."""
    parser = argparse.ArgumentParser(
        description='Scaffold a new robot descriptor configuration.'
    )
    parser.add_argument(
        '--robot_id',
        required=True,
        help='Unique identifier for the robot platform',
    )
    parser.add_argument(
        '--dof',
        type=int,
        default=6,
        help='Arm degrees of freedom placeholder count (default: 6)',
    )
    parser.add_argument(
        '--cameras',
        default='head',
        help='Comma-separated names of camera sensors (default: "head")',
    )
    parser.add_argument(
        '--mobile_base',
        choices=['omni', 'diff', 'none'],
        default='none',
        help='Type of mobile base platform (default: "none")',
    )
    parser.add_argument(
        '--output_dir',
        help='Override target output directory (default: package robots/ dir)',
    )
    parser.add_argument(
        '--gen_collection_config',
        action='store_true',
        help='Generate collection_config_<robot_id>.yaml alongside descriptor',
    )
    parser.add_argument(
        '--validate_only',
        action='store_true',
        help='Load and validate an existing robot descriptor only',
    )

    args = parser.parse_args()

    # If validate_only is selected, run parsing & validation immediately
    if args.validate_only:
        desc = None
        desc_path = None
        if args.output_dir:
            candidate = Path(args.output_dir) / f'{args.robot_id}.robot.yaml'
            if candidate.is_file():
                desc_path = candidate
                try:
                    desc = _parse_descriptor_file(candidate)
                except Exception as e:
                    print(
                        f"Error parsing descriptor file '{candidate}': {e}",
                        file=sys.stderr
                    )
                    sys.exit(1)

        if desc is None:
            try:
                desc = load_robot_descriptor(args.robot_id)
                desc_path = resolve_descriptor_path(args.robot_id)
            except FileNotFoundError as e:
                print(f'Error: {e}', file=sys.stderr)
                sys.exit(1)
            except Exception as e:
                print(
                    f"Error loading descriptor for '{args.robot_id}': {e}",
                    file=sys.stderr
                )
                sys.exit(1)

        errors = validate_descriptor(desc)
        placeholders = check_placeholders(desc_path) if desc_path else []
        if errors or placeholders:
            print(
                f"Validation failed for '{args.robot_id}':",
                file=sys.stderr
            )
            for err in errors:
                print(f'  - {err}', file=sys.stderr)
            for warn in placeholders:
                print(f'  - {warn}', file=sys.stderr)
            sys.exit(1)
        else:
            print(f"Robot descriptor '{args.robot_id}' is VALID.")
            sys.exit(0)

    # Determine target output directory
    output_dir = None
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        # Walk up file path to find package robots/ directory in source space
        current_file = Path(__file__).resolve()
        for parent in current_file.parents:
            if (
                parent.name == 'sobits_vla_common'
                and (parent / 'robots').is_dir()
                and 'install' not in parent.parts
            ):
                output_dir = parent / 'robots'
                break

        if output_dir is None:
            output_dir = Path.cwd()

    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)

    cameras_list = [c.strip() for c in args.cameras.split(',') if c.strip()]

    # Generate descriptor config
    desc_content = generate_descriptor_yaml(
        robot_id=args.robot_id,
        dof=args.dof,
        cameras=cameras_list,
        mobile_base=args.mobile_base,
    )
    desc_path = output_dir / f'{args.robot_id}.robot.yaml'
    with open(desc_path, 'w', encoding='utf-8') as f:
        f.write(desc_content)
    print(f'Generated robot descriptor: {desc_path}')

    # Generate collection config if requested
    if args.gen_collection_config:
        coll_content = generate_collection_config_yaml(args.robot_id)
        coll_path = output_dir / f'collection_config_{args.robot_id}.yaml'
        with open(coll_path, 'w', encoding='utf-8') as f:
            f.write(coll_content)
        print(f'Generated collection config stub: {coll_path}')

    # Run validation checks on generated config (will warn on TODOs)
    try:
        desc_obj = _parse_descriptor_file(desc_path)
        errors = validate_descriptor(desc_obj)
        placeholders = check_placeholders(desc_path)
        if errors or placeholders:
            print('\nGenerated config has warnings/TODOs that must be resolved:')
            for err in errors:
                print(f'  - {err}')
            for warn in placeholders:
                print(f'  - {warn}')
            print(
                '\nEdit the file, then re-run with --validate_only to confirm.'
            )
        else:
            print(f"\nGenerated robot descriptor '{args.robot_id}' is VALID.")
    except Exception as e:
        print(f'\nWarning: Failed to parse generated config: {e}')


if __name__ == '__main__':
    main()
