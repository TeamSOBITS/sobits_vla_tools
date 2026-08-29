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
Automatic episode runner for unattended VLA evaluation.

Drives the sobits_vla_deploy node through N episodes:
  1. Call the sobits_vla_deploy/command service with PLAY.
  2. Wait for sobits_vla_deploy/episode_done (published by the deploy node when an episode
     auto-terminates on success / fall / timeout, or on a manual stop).
  3. The deploy node resets the world on stop and publishes episode_done
     once the reset teleports have completed — the next PLAY follows
     immediately, no time-based settle.
  4. Repeat.

After the last episode the node logs a tally and shuts down so the launch
returns. One model per invocation (no hot-swap) — run it once per model.
"""

from __future__ import annotations

from collections import Counter
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sobits_interfaces.srv import VlaCommand
from std_msgs.msg import String
from tqdm import tqdm


class ExperimentRunner(Node):
    def __init__(self) -> None:
        super().__init__('vla_experiment_runner')

        self.declare_parameter('num_episodes', 20)
        self.declare_parameter('command_service', 'sobits_vla_deploy/command')
        self.declare_parameter('episode_done_topic', 'sobits_vla_deploy/episode_done')
        # Pause between settle and the next PLAY.
        self.declare_parameter('inter_episode_pause_s', 1.0)
        # Safety: how long to wait for an episode_done before forcing a STOP.
        # Should exceed task.common.episode_timeout_s with margin.
        self.declare_parameter('episode_timeout_s', 60.0)
        self.declare_parameter('done_wait_margin_s', 30.0)
        # Deploy node loads the policy (multi-GB download + GPU load) before
        # advertising the command service, so allow several minutes.
        self.declare_parameter('startup_timeout_s', 600.0)

        self._num_episodes = int(self.get_parameter('num_episodes').value)
        self._command_service = str(self.get_parameter('command_service').value)
        self._inter_episode_pause_s = float(
            self.get_parameter('inter_episode_pause_s').value
        )
        self._done_wait_s = (
            float(self.get_parameter('episode_timeout_s').value)
            + float(self.get_parameter('done_wait_margin_s').value)
        )
        self._startup_timeout_s = float(
            self.get_parameter('startup_timeout_s').value
        )
        done_topic = str(self.get_parameter('episode_done_topic').value)

        self._cli = self.create_client(VlaCommand, self._command_service)
        self._done_event = threading.Event()
        self._last_outcome = ''
        self._done_sub = self.create_subscription(
            String, done_topic, self._on_done, QoSProfile(depth=10)
        )

    def _on_done(self, msg: String) -> None:
        self._last_outcome = msg.data
        self._done_event.set()

    def _send_command(self, command: int, timeout: float = 10.0) -> bool:
        if not self._cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                'Service {} unavailable.'.format(self._command_service)
            )
            return False
        req = VlaCommand.Request()
        req.command = command
        future = self._cli.call_async(req)  # executor (other thread) drives it
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            self.get_logger().error('Command {} timed out.'.format(command))
            return False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error('Command {} raised: {}'.format(command, exc))
            return False
        if response is None or not response.success:
            self.get_logger().error(
                'Command {} failed: {}'.format(
                    command, response.message if response else 'no response'
                )
            )
            return False
        return True

    def _sleep(self, seconds: float) -> None:
        """Wall-clock sleep that aborts cleanly on rclpy shutdown."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))

    def _wait_done_sim_time(self, budget_s: float) -> bool:
        """
        Wait for episode_done for budget_s NODE-CLOCK seconds.

        With use_sim_time the node clock is simulation time, matching the
        deploy node's episode_timeout_s units, so RTF < 1 no longer makes
        this wait expire before the sim-time timeout can fire.
        """
        t0 = self.get_clock().now()
        while rclpy.ok():
            if self._done_event.wait(0.2):
                return True
            elapsed = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed >= budget_s:
                return False
        return False

    def run(self) -> None:
        self.get_logger().info(
            'Experiment runner: {} episodes, service={}, done-wait={}s.'.format(
                self._num_episodes, self._command_service, self._done_wait_s,
            )
        )
        # Block until the deploy node finishes loading its policy and
        # advertises the command service (multi-GB download on first run).
        self.get_logger().info(
            'Waiting up to {:.0f}s for {} (policy load) ...'.format(
                self._startup_timeout_s, self._command_service)
        )
        if not self._cli.wait_for_service(timeout_sec=self._startup_timeout_s):
            self.get_logger().error(
                'Deploy command service never appeared; aborting.'
            )
            return
        self.get_logger().info('Deploy service ready.')

        # A STOP while idle teleports robot+block to spawn pose; without this,
        # episode 1 would begin from a stale pose.
        self.get_logger().info('Resetting world to start pose before episode 1 ...')
        self._done_event.clear()
        self._send_command(VlaCommand.Request.STOP)
        # episode_done is published once the reset (teleports + pose motion)
        # has completed — no time-based settle needed.
        if not self._done_event.wait(60.0):
            self.get_logger().warning('Initial reset did not confirm within 60s; continuing.')
        self._sleep(self._inter_episode_pause_s)

        outcomes: Counter = Counter()
        progress = tqdm(total=self._num_episodes, desc='episodes', unit='ep',
                        disable=None)
        for ep in range(1, self._num_episodes + 1):
            if not rclpy.ok():
                break
            self._done_event.clear()
            self.get_logger().info('=== Episode {}/{}: PLAY ==='.format(
                ep, self._num_episodes))
            if not self._send_command(VlaCommand.Request.PLAY):
                self.get_logger().error('Aborting: could not start episode.')
                break

            # The deploy timeout counts SIM seconds; a wall-clock wait fires
            # early whenever RTF < 1, force-stopping before 'timeout' can.
            got = self._wait_done_sim_time(self._done_wait_s)
            if not got:
                self.get_logger().warning(
                    'Episode {} did not report done within {}s; forcing STOP.'
                    .format(ep, self._done_wait_s)
                )
                self._send_command(VlaCommand.Request.STOP)
                # The forced STOP publishes episode_done after the reset
                # completes.
                self._done_event.wait(30.0)
                outcome = self._last_outcome or 'forced_stop'
            else:
                outcome = self._last_outcome
            outcomes[outcome] += 1
            progress.set_postfix_str(outcome)
            progress.update(1)
            self.get_logger().info(
                'Episode {}/{} outcome: {}'.format(ep, self._num_episodes, outcome)
            )

            # episode_done already confirmed the reset completed; brief pause.
            self._sleep(self._inter_episode_pause_s)

        progress.close()
        self.get_logger().info(
            'Experiment complete. Outcomes: {}'.format(dict(outcomes))
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExperimentRunner()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
