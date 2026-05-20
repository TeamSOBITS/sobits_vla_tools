#!/usr/bin/env python3
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
Pre-flight VRAM estimator and gate.

Measures free GPU VRAM before training starts and aborts if the estimated
peak usage would exceed the configured limit (default 15.5 GB to leave
headroom under the 16 GB laptop GPU constraint).

Estimation strategy: known per-policy baseline footprints in POLICY_VRAM_GB,
scaled by batch size, plus current reserved memory and a system overhead constant.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

POLICY_VRAM_GB: dict[str, float] = {
    'smolvla': 8.0,
    'pi0': 13.0,
    'pi05': 13.5,
    'pi0_fast': 11.0,
}

_SYSTEM_OVERHEAD_GB = 0.8


def check_vram(
    policy_type: str,
    batch_size: int,
    limit_gb: float = 15.5,
    verbose: bool = True,
) -> None:
    """
    Assert that peak VRAM will fit within *limit_gb*.

    Parameters
    ----------
    policy_type : str
        One of the registered policy names.
    batch_size : int
        Training batch size (scales estimate linearly above 8).
    limit_gb : float
        Hard VRAM ceiling in gigabytes.
    verbose : bool
        Log estimate even when check passes.

    Raises
    ------
    RuntimeError
        If estimated peak VRAM exceeds *limit_gb* or no CUDA device is found.

    """
    try:
        import torch
    except ImportError as e:
        raise RuntimeError('PyTorch is required for VRAM estimation.') from e

    if not torch.cuda.is_available():
        logger.warning('No CUDA device found — skipping VRAM check (CPU run).')
        return

    device = torch.device('cuda')
    total_gb = torch.cuda.get_device_properties(device).total_memory / 1024 ** 3
    reserved_gb = torch.cuda.memory_reserved(device) / 1024 ** 3

    base_policy_gb = POLICY_VRAM_GB.get(policy_type, 10.0)
    activation_scale = max(1.0, batch_size / 8.0)
    estimated_gb = (base_policy_gb * activation_scale) + _SYSTEM_OVERHEAD_GB + reserved_gb

    if verbose or estimated_gb > limit_gb:
        logger.info(
            f'VRAM estimate | policy={policy_type} batch={batch_size} '
            f'estimated={estimated_gb:.1f} GB  limit={limit_gb:.1f} GB  '
            f'total_device={total_gb:.1f} GB'
        )

    if estimated_gb > limit_gb:
        raise RuntimeError(
            f'Estimated peak VRAM {estimated_gb:.1f} GB exceeds limit {limit_gb:.1f} GB '
            f"for policy '{policy_type}' with batch_size={batch_size}. "
            'Reduce batch_size, enable gradient_checkpointing, or use a smaller variant.'
        )

    if estimated_gb > total_gb:
        raise RuntimeError(
            f'Estimated peak VRAM {estimated_gb:.1f} GB exceeds device total '
            f'{total_gb:.1f} GB — training will OOM.'
        )

    logger.info(f'VRAM pre-flight passed ({estimated_gb:.1f} / {limit_gb:.1f} GB).')
