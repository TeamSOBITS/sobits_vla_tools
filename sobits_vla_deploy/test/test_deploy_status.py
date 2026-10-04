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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sobits_vla_deploy import deploy_status as ds  # noqa: E402
from sobits_vla_deploy.deploy_status import DeployStatus, format_elapsed  # noqa: E402


def _status(deadman=True):
    return DeployStatus('smolvla', deadman, 'pick')


def test_idle_snapshot():
    snap = _status().snapshot(0.0)
    assert (snap.state, snap.event, snap.event_seq) == (ds.STATE_STOPPED, ds.EVENT_NONE, 0)
    assert snap.message == 'idle' and snap.task_set and snap.task_name == 'pick'
    assert snap.policy == 'smolvla' and snap.episode_name == ''


def test_start_with_clock_running():
    st = _status(False)
    st.start(10.0, clock_running=True)
    snap = st.snapshot(12.5)
    assert (snap.state, snap.event, snap.episode_name) == (
        ds.STATE_PLAYING, ds.EVENT_STARTED, 'episode_1')
    assert snap.elapsed_sec == 2.5
    assert st.snapshot(13.0).event == ds.EVENT_NONE


def test_deferred_clock_starts_on_engagement():
    st = _status()
    st.start(10.0, clock_running=False)
    snap = st.snapshot(20.0)
    assert snap.elapsed_sec == 0.0 and snap.message == 'playing 00:00:00 (released)'
    st.engaged(21.0)
    snap = st.snapshot(24.0)
    assert snap.event == ds.EVENT_ENGAGED and snap.deadman_engaged
    assert snap.elapsed_sec == 3.0 and snap.message == 'playing 00:00:03'


def test_release_keeps_clock_running():
    st = _status()
    st.start(0.0, clock_running=False)
    st.engaged(1.0)
    st.released(5.0)
    snap = st.snapshot(7.0)
    assert snap.event == ds.EVENT_RELEASED and not snap.deadman_engaged
    assert snap.elapsed_sec == 6.0 and snap.message.endswith('(released)')


def test_stop_freezes_elapsed_and_resets_to_episode_done():
    st = _status(False)
    st.start(0.0, clock_running=True)
    st.stop(4.0, 'manual_stop')
    snap = st.snapshot(99.0)
    assert (snap.state, snap.event, snap.outcome) == (
        ds.STATE_RESETTING, ds.EVENT_STOPPED, 'manual_stop')
    assert snap.elapsed_sec == 4.0 and snap.detail == 'manual_stop' and snap.message == 'resetting'
    st.reset_done(30.0)
    snap = st.snapshot(31.0)
    assert (snap.state, snap.event, snap.outcome) == (
        ds.STATE_STOPPED, ds.EVENT_EPISODE_DONE, 'manual_stop')
    assert snap.elapsed_sec == 4.0 and snap.message == 'done: manual_stop'


def test_idle_reset_ends_with_reset_done_and_no_start_event():
    st = _status()
    st.resetting(1.0)
    snap = st.snapshot(1.0)
    assert (snap.state, snap.event, snap.event_seq) == (ds.STATE_RESETTING, ds.EVENT_NONE, 0)
    st.reset_done(5.0)
    snap = st.snapshot(5.0)
    assert (snap.state, snap.event, snap.message) == (
        ds.STATE_STOPPED, ds.EVENT_RESET_DONE, 'idle')


def test_idle_reset_drops_the_previous_outcome():
    st = _status(False)
    st.start(0.0, True)
    st.stop(1.0, 'timeout')
    st.reset_done(2.0)
    st.resetting(3.0)
    st.reset_done(4.0)
    snap = st.snapshot(4.0)
    assert (snap.outcome, snap.message) == ('', 'idle')


def test_reset_done_is_idempotent():
    st = _status()
    st.start(0.0, True)
    st.stop(1.0, 'timeout')
    st.reset_done(2.0)
    seq = st.snapshot(2.0).event_seq
    st.reset_done(3.0)
    assert st.snapshot(3.0).event_seq == seq


def test_start_clears_previous_episode():
    st = _status(False)
    st.start(0.0, True)
    st.step()
    st.stop(1.0, 'fallen')
    st.reset_done(2.0)
    st.start(3.0, True)
    snap = st.snapshot(3.0)
    assert (snap.steps, snap.outcome, snap.episode_name) == (0, '', 'episode_2')


def test_event_seq_counts_events_not_heartbeats():
    st = _status()
    assert st.snapshot(0.0).event_seq == 0
    st.set_task('place')
    assert st.snapshot(1.0).event_seq == 1
    assert st.snapshot(2.0).event_seq == 1
    st.reject('PAUSE not supported')
    snap = st.snapshot(3.0)
    assert (snap.event, snap.event_seq, snap.detail) == (
        ds.EVENT_REJECTED, 2, 'PAUSE not supported')
    assert snap.state == ds.STATE_STOPPED


def test_set_task_and_error():
    st = DeployStatus('p', False)
    assert not st.snapshot(0.0).task_set
    st.set_task('pick cup')
    snap = st.snapshot(0.0)
    assert (snap.event, snap.task_name, snap.detail) == (ds.EVENT_TASK_SET, 'pick cup', 'pick cup')
    st.error('boom')
    snap = st.snapshot(0.0)
    assert (snap.state, snap.event, snap.detail, snap.message) == (
        ds.STATE_ERROR, ds.EVENT_ERROR, 'boom', 'error')


def test_steps_count_and_reset_on_start():
    st = _status(False)
    st.start(0.0, True)
    for _ in range(3):
        st.step()
    assert st.snapshot(1.0).steps == 3


def test_inference_rate_window_and_staleness():
    st = _status(False)
    st.start(0.0, True)
    assert st.inference_hz(0.0) == 0.0
    st.chunk(1.0)
    assert st.inference_hz(1.0) == 0.0
    for t in (1.5, 2.0, 2.5):
        st.chunk(t)
    assert st.inference_hz(2.5) == 2.0
    assert st.inference_hz(8.0) == 0.0


def test_inference_rate_zero_when_not_playing():
    st = _status(False)
    st.start(0.0, True)
    st.chunk(1.0)
    st.chunk(2.0)
    st.stop(2.0, 'manual_stop')
    assert st.snapshot(2.0).inference_hz == 0.0


def test_format_elapsed():
    assert format_elapsed(0) == '00:00:00'
    assert format_elapsed(3725.9) == '01:02:05'
    assert format_elapsed(-4) == '00:00:00'
