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

"""Shared ROS image/compressed-image message decoding, cv_bridge-free."""

import io

import cv2
import numpy as np

try:
    from PIL import Image as PILImage
except Exception:
    PILImage = None


def decode_image_message(msg) -> np.ndarray | None:
    """Decode ROS image/compressed-image message into RGB uint8 HWC array."""
    # CompressedImage-like messages usually expose a `format` field.
    if hasattr(msg, 'format'):
        data = msg.data
        if isinstance(data, memoryview):
            data = data.tobytes()
        if isinstance(data, (bytes, bytearray)):
            encoded = np.frombuffer(data, dtype=np.uint8)
        else:
            # rosbags can surface sequence payloads as Python lists/arrays.
            encoded = np.asarray(data, dtype=np.uint8)

        if encoded.size == 0:
            return None

        # Preferred path: OpenCV decode (fast). Some environments can fail due
        # OpenCV/Numpy ABI mismatch, so we fall back to Pillow decode.
        try:
            img = cv2.imdecode(np.ascontiguousarray(encoded), cv2.IMREAD_UNCHANGED)
        except Exception:
            img = None

        if img is not None:
            if img.ndim == 2:
                return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            if img.shape[2] == 4:
                return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        if PILImage is None:
            return None
        try:
            pil_img = PILImage.open(io.BytesIO(encoded.tobytes())).convert('RGB')
            return np.asarray(pil_img, dtype=np.uint8)
        except Exception:
            return None

    encoding = getattr(msg, 'encoding', '')
    data = msg.data
    if isinstance(data, memoryview):
        data = bytes(data)
    raw = np.frombuffer(data, dtype=np.uint8)
    h, w = msg.height, msg.width
    if encoding in ('mono8', '8UC1'):
        img = raw.reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if encoding in ('mono16', '16UC1'):
        img = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
        img8 = (img >> 8).astype(np.uint8)
        return cv2.cvtColor(img8, cv2.COLOR_GRAY2RGB)
    if encoding in ('rgb8',):
        return raw.reshape(h, w, 3).copy()
    if encoding in ('bgr8',):
        img = raw.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if encoding in ('rgba8',):
        img = raw.reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    if encoding in ('bgra8',):
        img = raw.reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    # Fallback: try to reshape as BGR and convert
    channels = len(raw) // (h * w) if h * w > 0 else 3
    img = raw.reshape(h, w, channels)
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    if img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    return img


def decode_depth_message(msg) -> np.ndarray | None:
    """Decode ROS 16UC1 depth message into raw uint16 millimetre HxW array."""
    if hasattr(msg, 'format'):
        data = msg.data
        if isinstance(data, memoryview):
            data = data.tobytes()
        if isinstance(data, (bytes, bytearray)):
            encoded = np.frombuffer(data, dtype=np.uint8)
        else:
            encoded = np.asarray(data, dtype=np.uint8)

        if encoded.size == 0:
            return None

        # compressedDepth is a plain 16-bit PNG; IMREAD_UNCHANGED keeps depth.
        try:
            img = cv2.imdecode(np.ascontiguousarray(encoded), cv2.IMREAD_UNCHANGED)
        except Exception:
            img = None

        if img is not None and img.dtype == np.uint16:
            return img if img.ndim == 2 else img[..., 0]

        if PILImage is None:
            return None
        try:
            pil_img = PILImage.open(io.BytesIO(encoded.tobytes()))
            arr = np.asarray(pil_img)
            return arr.astype(np.uint16) if arr.dtype != np.uint16 else arr
        except Exception:
            return None

    # No bit-shifting here, unlike decode_image_message — preserves full depth range.
    encoding = getattr(msg, 'encoding', '')
    data = msg.data
    if isinstance(data, memoryview):
        data = bytes(data)
    h, w = msg.height, msg.width
    if encoding in ('16UC1', 'mono16'):
        return np.frombuffer(data, dtype=np.uint16).reshape(h, w).copy()
    return None
