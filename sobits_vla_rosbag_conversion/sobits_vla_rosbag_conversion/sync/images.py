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

"""RGB + depth frame selection/decoding: per-camera nearest matching and decode."""

import bisect

from sobits_vla_common.image_codec import decode_depth_message, decode_image_message


def get_nearest_image(series, target_time, times=None):
    """Get the nearest image (msg, rawdata, connection, t) to target_time in the series."""
    if not series:
        return None, None, None, 0.0
    if len(series) == 1:
        # entry is (t_sec, msg, rawdata, connection)
        return series[0][1], series[0][2], series[0][3], series[0][0]

    if times is None:
        times = [s[0] for s in series]
    idx = bisect.bisect_right(times, target_time)

    if idx == 0:
        return series[0][1], series[0][2], series[0][3], series[0][0]
    if idx == len(series):
        return series[-1][1], series[-1][2], series[-1][3], series[-1][0]

    t_prev, msg_prev, raw_prev, conn_prev = series[idx - 1]
    t_next, msg_next, raw_next, conn_next = series[idx]

    if abs(target_time - t_prev) < abs(target_time - t_next):
        return msg_prev, raw_prev, conn_prev, t_prev
    else:
        return msg_next, raw_next, conn_next, t_next


def decode_primary(msg_prim):
    """Decode the primary camera image; returns None on any decode failure."""
    try:
        return decode_image_message(msg_prim)
    except Exception:
        return None


def decode_secondary_images(camera_topics, primary_camera, cam_series, cam_series_times, t_sec):
    """
    Fetch + decode every non-primary RGB camera at nearest time to t_sec.

    Returns (images, image_times, failed) -- failed=True aborts the frame,
    mirroring the original inline break-on-first-failure loop.
    """
    images = {}
    image_times = {}
    for cam_name in camera_topics.keys():
        if cam_name == primary_camera:
            continue
        msg_img, _raw_img, _conn_img, t_img = get_nearest_image(
            cam_series[cam_name], t_sec, times=cam_series_times[cam_name]
        )
        if msg_img is None:
            return images, image_times, True
        try:
            img = decode_image_message(msg_img)
            if img is None:
                return images, image_times, True
            images[cam_name] = img
            image_times[cam_name] = t_img
        except Exception:
            return images, image_times, True
    return images, image_times, False


def decode_depth_images(depth_camera_topics, cam_series, cam_series_times, t_sec):
    """Fetch + decode every depth camera at nearest time to t_sec, same pattern as RGB."""
    depth_images = {}
    image_times = {}
    for cam_name in depth_camera_topics.keys():
        msg_img, _raw_img, _conn_img, t_img = get_nearest_image(
            cam_series[cam_name], t_sec, times=cam_series_times[cam_name]
        )
        if msg_img is None:
            return depth_images, image_times, True
        try:
            depth_img = decode_depth_message(msg_img)
            if depth_img is None:
                return depth_images, image_times, True
            depth_images[cam_name] = depth_img
            image_times[cam_name] = t_img
        except Exception:
            return depth_images, image_times, True
    return depth_images, image_times, False
