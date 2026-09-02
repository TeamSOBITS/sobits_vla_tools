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
Figure registry.

FIGURES maps name -> plotting function, so adding a plot means adding a
module here, not editing cli.py's orchestration.
"""

from typing import Callable, Dict

from sobits_vla_deploy.eval.figures.duration import fig_duration
from sobits_vla_deploy.eval.figures.economy import fig_economy
from sobits_vla_deploy.eval.figures.ee_trajectory import fig_ee_trajectory
from sobits_vla_deploy.eval.figures.jerk import fig_jerk
from sobits_vla_deploy.eval.figures.outcomes import fig_outcomes
from sobits_vla_deploy.eval.figures.scores import fig_scores
from sobits_vla_deploy.eval.figures.stage_funnel import fig_stage_funnel
from sobits_vla_deploy.eval.figures.timeseries import fig_timeseries

# Signatures vary (per_ep-only, models-only, or models+column+...), so cli.py
# calls each entry with the args its own docstring/name implies -- this
# registry exists so a new figure is one import + one dict entry, not a
# cli.py edit.
FIGURES: Dict[str, Callable] = {
    'operator_scores': fig_scores,
    'stage_funnel': fig_stage_funnel,
    'outcomes': fig_outcomes,
    'duration': fig_duration,
    'tracking_error': fig_timeseries,
    'joint_jerk': fig_jerk,
    'motion_economy': fig_economy,
    'ee_trajectory': fig_ee_trajectory,
}
