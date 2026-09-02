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
Metadata-consistency checks and camera topic/shape resolution.

Pure functions over plain dicts, no Node/rclpy. Logic moved verbatim from
RosbagConversionNode's _morphologies_match/_sensors_match/_resolve_cameras;
callers pass a duck-typed logger (get_logger()-shaped) or None.
"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from rosbags.highlevel import AnyReader
from tqdm import tqdm


def _log(logger, level: str, msg: str) -> None:
    if logger is not None:
        getattr(logger, level)(msg)


def morphologies_match(ref_morph: dict, other_morph: dict) -> bool:
    """Return True if the two morphology dicts are functionally equivalent."""
    for key in ['type', 'has_mobile_base', 'has_cmd_vel_y', 'joint_states_topic']:
        if ref_morph.get(key) != other_morph.get(key):
            return False
    ref_parts = ref_morph.get('parts', [])
    other_parts = other_morph.get('parts', [])
    if ref_parts != other_parts:
        return False
    for part in ref_parts:
        rp = ref_morph.get(part, {})
        op = other_morph.get(part, {})
        if rp.get('is_actionable') != op.get('is_actionable'):
            return False
        if rp.get('joint_names', []) != op.get('joint_names', []):
            return False
    return True


def sensors_match(ref_sensors: dict, other_sensors: dict) -> bool:
    """Return True if the two sensors blocks declare the same topics/shapes."""
    ref_types = ref_sensors.get('types', [])
    other_types = other_sensors.get('types', [])
    if sorted(ref_types) != sorted(other_types):
        return False
    for stype in ref_types:
        rs = ref_sensors.get(stype, {})
        os_ = other_sensors.get(stype, {})
        for key in ['names', 'topics', 'compressed_topics', 'info_topics']:
            if rs.get(key, []) != os_.get(key, []):
                return False
        if rs.get('properties', {}) != os_.get('properties', {}):
            return False
    return True


def resolve_cameras(
    selected_names: List[str],
    selected_compressed: List[bool],
    all_cam_known: set,
    all_cam_raw: Dict[str, str],
    all_cam_compressed: Dict[str, str],
    all_cam_info: Dict[str, str],
    all_cam_props: Dict[str, dict],
    candidate_bag_dirs: List[str],
    decode_fn,
    label: str,
    logger=None,
) -> Optional[Tuple[Dict[str, str], Dict[str, str], Dict[str, Tuple[int, int]]]]:
    """Resolve topics/info_topics/shapes for one camera group (RGB or depth)."""
    topics, info_topics = _resolve_topics(
        selected_names, selected_compressed, all_cam_known,
        all_cam_raw, all_cam_compressed, all_cam_info, label, logger,
    )
    if topics is None:
        return None

    shapes, unresolved_set = _shapes_from_properties(topics, all_cam_props)
    if unresolved_set:
        _fill_shapes_from_camera_info(shapes, unresolved_set, info_topics, candidate_bag_dirs)
    if unresolved_set:
        _fill_shapes_from_images(shapes, unresolved_set, topics, candidate_bag_dirs, decode_fn)

    if unresolved_set:
        _log_unresolved(unresolved_set, info_topics, topics, label, logger)
        return None
    return topics, info_topics, shapes


def _resolve_topics(
    selected_names, selected_compressed, all_cam_known,
    all_cam_raw, all_cam_compressed, all_cam_info, label, logger,
):
    if len(selected_compressed) < len(selected_names):
        selected_compressed = selected_compressed + [False] * (
            len(selected_names) - len(selected_compressed)
        )

    topics: Dict[str, str] = {}
    info_topics: Dict[str, str] = {}
    for name, use_compressed in zip(selected_names, selected_compressed):
        if name not in all_cam_known:
            _log(logger, 'error', f"{label} '{name}' not found in metadata sensors. Aborting.")
            return None, None
        if use_compressed:
            topic = all_cam_compressed.get(name, '')
            if not topic:
                _log(
                    logger, 'warning',
                    f"{label} '{name}': no compressed topic in metadata, falling back to raw.",
                )
                topic = all_cam_raw.get(name, '')
        else:
            topic = all_cam_raw.get(name, '')
        if not topic:
            _log(
                logger, 'error',
                f"{label} '{name}': no topic available (raw or compressed). Aborting.",
            )
            return None, None
        topics[name] = topic
        info_topics[name] = all_cam_info.get(name, '')
    return topics, info_topics


def _shapes_from_properties(topics, all_cam_props):
    shapes: Dict[str, Tuple[int, int]] = {}
    unresolved = []
    for cam_name in topics.keys():
        props = all_cam_props.get(cam_name, {})
        width = props.get('width')
        height = props.get('height')
        if width and height:
            shapes[cam_name] = (int(width), int(height))
        else:
            unresolved.append(cam_name)
    return shapes, set(unresolved)


def _fill_shapes_from_camera_info(shapes, unresolved_set, info_topics, candidate_bag_dirs):
    # First pass over the bags: only runs when metadata lacks the shapes.
    for bag_dir in tqdm(candidate_bag_dirs, desc='sniffing camera_info',
                        unit='bag', disable=None, leave=False):
        if not unresolved_set:
            break
        try:
            with AnyReader([Path(bag_dir)]) as reader:
                info_topic_to_cam = {
                    info_topics[name]: name
                    for name in unresolved_set
                    if info_topics.get(name)
                }
                if not info_topic_to_cam:
                    break
                connections = [c for c in reader.connections if c.topic in info_topic_to_cam]
                for connection, _, rawdata in reader.messages(connections=connections):
                    cam_name = info_topic_to_cam.get(connection.topic)
                    if not cam_name or cam_name not in unresolved_set:
                        continue
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    shapes[cam_name] = (int(msg.width), int(msg.height))
                    unresolved_set.remove(cam_name)
                    if not unresolved_set:
                        break
        except Exception:
            continue


def _fill_shapes_from_images(shapes, unresolved_set, topics, candidate_bag_dirs, decode_fn):
    for bag_dir in tqdm(candidate_bag_dirs, desc='sniffing image shapes',
                        unit='bag', disable=None, leave=False):
        if not unresolved_set:
            break
        try:
            with AnyReader([Path(bag_dir)]) as reader:
                topic_to_cam = {
                    topics[name]: name for name in unresolved_set if topics.get(name)
                }
                if not topic_to_cam:
                    break
                connections = [c for c in reader.connections if c.topic in topic_to_cam]
                for connection, _, rawdata in reader.messages(connections=connections):
                    cam_name = topic_to_cam.get(connection.topic)
                    if not cam_name or cam_name not in unresolved_set:
                        continue
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    try:
                        img = decode_fn(msg)
                        if img is None:
                            continue
                    except Exception:
                        continue
                    h, w = img.shape[:2]
                    shapes[cam_name] = (int(w), int(h))
                    unresolved_set.remove(cam_name)
                    if not unresolved_set:
                        break
        except Exception:
            continue


def _log_unresolved(unresolved_set, info_topics, topics, label, logger):
    unresolved_msgs = []
    for cam_name in sorted(unresolved_set):
        info_topic = info_topics.get(cam_name, '')
        image_topic = topics.get(cam_name, '')
        unresolved_msgs.append(
            f"camera='{cam_name}', info_topic='{info_topic}', image_topic='{image_topic}'"
        )
    _log(
        logger, 'error',
        f'Failed to resolve {label.lower()} dimensions from metadata, '
        'camera_info, or image streams. '
        'Please re-record metadata with sensor properties populated.',
    )
    for msg in unresolved_msgs:
        _log(logger, 'error', f'  unresolved: {msg}')
