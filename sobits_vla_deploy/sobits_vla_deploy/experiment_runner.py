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
  1. Call the /vla/command service with PLAY.
  2. Wait for /vla/episode_done (published by the deploy node when an episode
     auto-terminates on success / fall / timeout, or on a manual stop).
  3. The deploy node resets the world on stop; wait reset_settle_s for the
     teleport + detecting_pose move to settle.
  4. Repeat.

After the last episode the node logs a tally and shuts down so the launch
returns. One model per invocation (no hot-swap) — run it once per model.
"""

from __future__ import annotations

import threading
import time
from collections import Counter

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sobits_interfaces.srv import VlaCommand
from std_msgs.msg import String


class ExperimentRunner(Node):
    def __init__(self) -> None:
        super().__init__('vla_experiment_runner')

        self.declare_parameter('num_episodes', 20)
        self.declare_parameter('command_service', '/vla/command')
        self.declare_parameter('episode_done_topic', '/vla/episode_done')
        # Time to let the world reset + detecting_pose move settle after a stop.
        self.declare_parameter('reset_settle_s', 6.0)
        # Pause between settle and the next PLAY.
        self.declare_parameter('inter_episode_pause_s', 1.0)
        # Safety: how long to wait for an episode_done before forcing a STOP.
        # Should exceed logging.episode_timeout_s with margin.
        self.declare_parameter('episode_timeout_s', 60.0)
        self.declare_parameter('done_wait_margin_s', 30.0)
        # How long to wait for the deploy node's command service to appear.
        # The deploy node loads the policy (a multi-GB download on first run +
        # GPU load) before advertising the service, so allow several minutes.
        self.declare_parameter('startup_timeout_s', 600.0)

        self._num_episodes = int(self.get_parameter('num_episodes').value)
        self._command_service = str(self.get_parameter('command_service').value)
        self._reset_settle_s = float(self.get_parameter('reset_settle_s').value)
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
        return True

    def _sleep(self, seconds: float) -> None:
        """Wall-clock sleep that aborts cleanly on rclpy shutdown."""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))

    def run(self) -> None:
        self.get_logger().info(
            'Experiment runner: {} episodes, service={}, done-wait={}s, '
            'settle={}s.'.format(
                self._num_episodes, self._command_service,
                self._done_wait_s, self._reset_settle_s,
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

        # Reset the world before episode 1 so the first episode starts from the
        # spawn pose (a STOP while idle teleports robot+block + moves to
        # initial_pose). Without this, episode 1 begins from a stale pose.
        self.get_logger().info('Resetting world to start pose before episode 1 ...')
        self._send_command(VlaCommand.Request.STOP)
        self._sleep(self._reset_settle_s + self._inter_episode_pause_s)

        outcomes: Counter = Counter()
        for ep in range(1, self._num_episodes + 1):
            if not rclpy.ok():
                break
            self._done_event.clear()
            self.get_logger().info('=== Episode {}/{}: PLAY ==='.format(
                ep, self._num_episodes))
            if not self._send_command(VlaCommand.Request.PLAY):
                self.get_logger().error('Aborting: could not start episode.')
                break

            got = self._done_event.wait(self._done_wait_s)
            if not got:
                self.get_logger().warn(
                    'Episode {} did not report done within {}s; forcing STOP.'
                    .format(ep, self._done_wait_s)
                )
                self._send_command(VlaCommand.Request.STOP)
                # The forced STOP itself publishes episode_done; consume it.
                self._done_event.wait(5.0)
                outcome = self._last_outcome or 'forced_stop'
            else:
                outcome = self._last_outcome
            outcomes[outcome] += 1
            self.get_logger().info(
                'Episode {}/{} outcome: {}'.format(ep, self._num_episodes, outcome)
            )

            # Let the deploy node's world reset + detecting_pose settle.
            self._sleep(self._reset_settle_s + self._inter_episode_pause_s)

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
