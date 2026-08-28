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

"""Run-scoped config threaded through eval/, replacing the old module-level global."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Set

# Group names excluded from the arm tracking-error metric. Hands/grippers and
# the head are not what the policy is judged on and skew the mean.
NON_ARM_GROUP_HINTS = ('hand', 'gripper', 'finger', 'head', 'end_effector')


@dataclass
class EvalContext:
    """
    Per-run config, replacing the old ARM_GROUPS module global.

    arm_groups: explicit --arm-groups selection; None = derive per episode
    from its own joint_groups meta (see resolve_arm_groups).
    """

    arm_groups: Optional[Set[str]] = field(default=None)

    def resolve_arm_groups(self, joint_groups) -> Optional[Set[str]]:
        """
        Pick the groups whose tracking error represents the arm.

        An explicit --arm-groups wins. Otherwise every group is counted
        except hands/grippers/head, matching the intent of the old prefix
        filter but driven by the descriptor's own group names.
        """
        if self.arm_groups is not None:
            return set(self.arm_groups)
        if not joint_groups:
            return None
        return {
            g for g in joint_groups
            if not any(h in g.lower() for h in NON_ARM_GROUP_HINTS)
        } or None
