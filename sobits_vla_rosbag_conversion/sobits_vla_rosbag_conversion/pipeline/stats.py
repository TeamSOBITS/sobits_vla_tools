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
Builds the conversion_stats.yaml dict from per-episode results.

No rclpy import. Key order matches DatasetWriter.finalize()'s original
inline dict exactly -- yaml.dump(sort_keys=False) makes order observable.
"""

from typing import Any, Dict, List

from sobits_vla_rosbag_conversion.pipeline.episode_pipeline import EpisodeResult


def episode_stat_dict(result: EpisodeResult) -> Dict[str, Any]:
    """Build the per-episode entry as it appears under stats['episodes']."""
    d = {
        'bag': result.bag,
        'task': result.task,
        'frames': result.frames,
        'skipped_downsample': result.skipped_downsample,
        'skipped_static': result.skipped_static,
        'skipped_tf': result.skipped_tf,
        'skipped_img_decode': result.skipped_img_decode,
    }
    if result.sync_avg_ms is not None:
        d['sync_avg_ms'] = result.sync_avg_ms
        d['sync_max_ms'] = result.sync_max_ms
        d['sync_min_ms'] = result.sync_min_ms
    return d


def build_stats_report(
    dataset_name: str,
    conversion_params: Dict[str, Any],
    episode_results: List[EpisodeResult],
    skipped_bags: List[Dict[str, Any]],
    fps_warnings: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Assemble the dict dumped as conversion_stats.yaml; key order is load-bearing."""
    total_episodes = len(episode_results)
    total_frames = sum(r.frames for r in episode_results)
    return {
        'dataset_name': dataset_name,
        **conversion_params,
        'total_episodes': total_episodes,
        'total_frames': total_frames,
        'skipped_bags': skipped_bags if skipped_bags else [],
        'fps_warnings': fps_warnings if fps_warnings else [],
        'episodes': [episode_stat_dict(r) for r in episode_results],
    }
