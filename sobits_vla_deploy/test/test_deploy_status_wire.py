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

import os
import sys

from builtin_interfaces.msg import Time
import pytest
from sobits_interfaces.msg import VlaStatus

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy import deploy_status as ds  # noqa: E402
from sobits_vla_deploy.deploy_status_feed import to_msg  # noqa: E402


@pytest.mark.parametrize('name', [
    'STATE_STOPPED', 'STATE_PLAYING', 'STATE_ERROR', 'STATE_RESETTING', 'EVENT_NONE',
    'EVENT_STARTED', 'EVENT_ERROR', 'EVENT_TASK_SET', 'EVENT_REJECTED', 'EVENT_STOPPED',
    'EVENT_EPISODE_DONE', 'EVENT_ENGAGED', 'EVENT_RELEASED', 'EVENT_RESET_DONE'])
def test_constants_match_message(name):
    assert getattr(ds, name) == getattr(VlaStatus, name)


def test_to_msg_maps_every_field():
    tracker = ds.DeployStatus('smolvla', True, 'pick cup')
    tracker.start(0.0, clock_running=False)
    tracker.engaged(1.0)
    tracker.step()
    tracker.chunk(1.0)
    tracker.chunk(1.5)
    msg = to_msg(tracker.snapshot(3.0), Time(sec=7, nanosec=9))
    assert (msg.stamp.sec, msg.stamp.nanosec) == (7, 9)
    assert msg.stage == VlaStatus.STAGE_DEPLOY
    assert (msg.state, msg.event, msg.event_seq) == (
        VlaStatus.STATE_PLAYING, VlaStatus.EVENT_ENGAGED, 2)
    assert msg.task_set and msg.task_name == 'pick cup' and msg.episode_name == 'episode_1'
    assert msg.elapsed_sec == pytest.approx(2.0)
    assert msg.message == 'playing 00:00:02' and msg.detail == ''
    assert msg.policy == 'smolvla' and msg.deadman_enabled and msg.deadman_engaged
    assert msg.steps == 1 and msg.inference_hz == pytest.approx(2.0) and msg.outcome == ''


def test_to_msg_stop_carries_outcome():
    tracker = ds.DeployStatus('p', False)
    tracker.start(0.0, clock_running=True)
    tracker.stop(1.0, 'manual_stop')
    msg = to_msg(tracker.snapshot(1.0), Time())
    assert msg.event == VlaStatus.EVENT_STOPPED and msg.state == VlaStatus.STATE_RESETTING
    assert msg.outcome == 'manual_stop' and msg.detail == 'manual_stop'
