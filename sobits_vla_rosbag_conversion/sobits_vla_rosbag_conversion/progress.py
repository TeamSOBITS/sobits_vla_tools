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


"""tqdm bars that degrade to log lines under ros2 launch."""

import os
import re
import sys

from tqdm import tqdm

_ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')

# ros2 launch relays child output one complete line at a time, so a \r-refreshed
# bar never shows. The launch file sets this to 'lines' to get one line per refresh.
PROGRESS_ENV = 'SOBITS_VLA_PROGRESS'


class _LineFile:
    """Write each tqdm refresh as a newline-terminated line on stderr."""

    def write(self, s):
        # Nested bars also emit cursor moves (\n, ESC[A); drop those, keep bar text.
        s = _ANSI.sub('', s).replace('\r', '').strip()
        if s:
            sys.stderr.write(s + '\n')

    def flush(self):
        sys.stderr.flush()


def line_mode() -> bool:
    return os.environ.get(PROGRESS_ENV, '') == 'lines'


def bar_mode() -> bool:
    """Real in-place bars: stderr is a terminal and nothing asked for lines."""
    return not line_mode() and sys.stderr.isatty()


class TqdmLogger:
    """rclpy-logger look-alike that prints through tqdm.write so bars stay at the bottom.

    In bar mode the node runs with --disable-stdout-logs, so this is the console;
    the real logger still receives every message (log file, rosout).
    """

    def __init__(self, logger):
        self._logger = logger

    def _console(self, level: str, msg: str):
        if bar_mode():
            tqdm.write(f'[{level}] {msg}', file=sys.stderr)

    # rclpy pins one severity per call site, so every level calls the logger on its own line.
    def debug(self, msg):
        self._logger.debug(msg)
        self._console('DEBUG', msg)

    def info(self, msg):
        self._logger.info(msg)
        self._console('INFO', msg)

    def warning(self, msg):
        self._logger.warning(msg)
        self._console('WARNING', msg)

    warn = warning

    def error(self, msg):
        self._logger.error(msg)
        self._console('ERROR', msg)

    def fatal(self, msg):
        self._logger.fatal(msg)
        self._console('FATAL', msg)


def progress(iterable=None, **kwargs):
    """tqdm(...) as usual; in line mode, one plain line every `mininterval` (default 10 s)."""
    if line_mode():
        kwargs.setdefault('mininterval', 10.0)
        # delay: phases shorter than the interval print nothing, not a lone 0% line.
        kwargs.setdefault('delay', kwargs['mininterval'])
        kwargs.update(file=_LineFile(), disable=False, leave=False, ascii=True, ncols=80,
                      dynamic_ncols=False, position=0)
    else:
        kwargs.setdefault('disable', None)  # auto-off when stderr is not a TTY
    return tqdm(iterable, **kwargs)
