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

"""EE-pose synthesis via TF lookups, per-axis unwrap (fb188e1), and delta."""

import numpy as np
from sobits_vla_rosbag_conversion.offline_tf_tree import mat_to_pose6d

# Not sobits_vla_common.gz_utils.wrap_pi: that wraps an absolute angle to
# (-pi, pi]; this unwraps a delta against the previous sample (fb188e1).


def resolve_ee_pose(tf_tree, ee_src, ee_tgt, stamp_ns):
    """Look up one EE transform; returns a 4x4 matrix or None on failure."""
    return tf_tree.resolve(ee_tgt, ee_src, stamp_ns)


def compute_ee_pose_and_delta(ee_mat, prev):
    """
    Convert a TF matrix to abs pose6d + delta vs the previous sample.

    Unwraps each rotation axis against prev before differencing, so a
    genuine continuous rotation crossing +-pi doesn't alias into a huge
    single-step jump in the delta (fb188e1) -- must not be replaced by a
    plain wrap-to-range helper, the two are not equivalent.
    """
    ee_abs = mat_to_pose6d(ee_mat)
    if prev is not None:
        for ax in range(3, 6):
            diff = ee_abs[ax] - prev[ax]
            if diff > np.pi:
                ee_abs[ax] -= 2 * np.pi
            elif diff < -np.pi:
                ee_abs[ax] += 2 * np.pi
        ee_rel = ee_abs - prev
    else:
        ee_rel = np.zeros(6, dtype=np.float32)
    return ee_abs, ee_rel
