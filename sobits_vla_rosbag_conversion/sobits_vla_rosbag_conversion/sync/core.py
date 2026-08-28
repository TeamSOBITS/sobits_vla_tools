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

"""Modality-agnostic time-grid/resampling machinery shared by every sync/ module."""

import bisect


def interpolate_vector(series, target_time, dim, times=None):
    """Linearly interpolates a time-series list of (t, list_of_floats) at target_time."""
    if not series:
        return [0.0] * dim
    if len(series) == 1:
        return series[0][1]

    if times is None:
        times = [s[0] for s in series]
    idx = bisect.bisect_right(times, target_time)

    if idx == 0:
        return series[0][1]
    if idx == len(series):
        return series[-1][1]

    t_prev, val_prev = series[idx - 1]
    t_next, val_next = series[idx]

    dt = t_next - t_prev
    if dt <= 0.0:
        return val_prev

    alpha = (target_time - t_prev) / dt
    return [(1.0 - alpha) * val_prev[j] + alpha * val_next[j] for j in range(dim)]


def interpolate_dict(series, target_time, keys, times=None):
    """Linearly interpolates a time-series list of (t, dict_of_floats) at target_time."""
    if not series:
        return {k: 0.0 for k in keys}
    if len(series) == 1:
        return {k: series[0][1].get(k, 0.0) for k in keys}

    if times is None:
        times = [s[0] for s in series]
    idx = bisect.bisect_right(times, target_time)

    if idx == 0:
        return {k: series[0][1].get(k, 0.0) for k in keys}
    if idx == len(series):
        return {k: series[-1][1].get(k, 0.0) for k in keys}

    t_prev, dict_prev = series[idx - 1]
    t_next, dict_next = series[idx]

    dt = t_next - t_prev
    if dt <= 0.0:
        return {k: dict_prev.get(k, 0.0) for k in keys}

    alpha = (target_time - t_prev) / dt
    res = {}
    for k in keys:
        v_prev = dict_prev.get(k, 0.0)
        v_next = dict_next.get(k, 0.0)
        res[k] = (1.0 - alpha) * v_prev + alpha * v_next
    return res


def hold_dict(series, target_time, keys, times=None):
    """
    Zero-order hold of a (t, dict_of_floats) series at target_time.

    Commanded positions are discrete set-points, not samples of a
    continuous signal: a trajectory point stays in force until the next
    command arrives. Interpolating between two commands invents motion
    that was never commanded -- e.g. a grasp held closed for 11 s reads
    back as the hand slowly reopening across the whole hold.

    Returns the most recent command at or before *target_time*.
    """
    if not series:
        return {k: 0.0 for k in keys}

    if times is None:
        times = [s[0] for s in series]
    idx = bisect.bisect_right(times, target_time) - 1
    if idx < 0:
        # target_time precedes every command; the first one is the best
        # available estimate of the set-point in force.
        idx = 0
    return {k: series[idx][1].get(k, 0.0) for k in keys}


def any_arrived_in_interval(series, t_start, t_end, times=None):
    """Check if any message in the series arrived in the interval (t_start, t_end]."""
    if not series:
        return False
    if times is None:
        times = [s[0] for s in series]
    idx = bisect.bisect_right(times, t_start)
    if idx < len(times) and times[idx] <= t_end:
        return True
    return False


def get_closest_t(series, target, times=None):
    if not series:
        return target
    if times is None:
        times = [s[0] for s in series]
    idx = bisect.bisect_left(times, target)
    if idx == 0:
        return times[0]
    if idx == len(times):
        return times[-1]
    if abs(times[idx] - target) < abs(times[idx - 1] - target):
        return times[idx]
    return times[idx - 1]


def should_downsample(t_sec, last_frame_time, min_frame_interval, downsample_tolerance):
    # Enforce minimum interval between frames, with tolerance for jitter.
    if min_frame_interval <= 0.0 or last_frame_time <= 0.0:
        return False
    return (t_sec - last_frame_time) < (min_frame_interval - downsample_tolerance)
