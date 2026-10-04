// Copyright (c) 2026, Team SOBITS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "sobits_vla_rosbag_collection/image_header_size.hpp"

#include <cstddef>

namespace sobits_vla
{

namespace
{

// compressed_depth_image_transport prepends a ConfigHeader (format + 2 floats).
constexpr size_t kCompressedDepthHeaderSize = 12;

uint32_t readBe16(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 8) | p[1];
}

uint32_t readBe32(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | p[3];
}

bool pngSize(const uint8_t * p, size_t n, uint32_t & width, uint32_t & height)
{
  static const uint8_t kSig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  // Signature, IHDR length, "IHDR", then width and height.
  if (n < 24) {return false;}
  for (size_t i = 0; i < 8; ++i) {
    if (p[i] != kSig[i]) {return false;}
  }
  if (p[12] != 'I' || p[13] != 'H' || p[14] != 'D' || p[15] != 'R') {return false;}
  width = readBe32(p + 16);
  height = readBe32(p + 20);
  return width > 0 && height > 0;
}

bool jpegSize(const uint8_t * p, size_t n, uint32_t & width, uint32_t & height)
{
  if (n < 4 || p[0] != 0xFF || p[1] != 0xD8) {return false;}
  size_t pos = 2;
  while (pos + 1 < n) {
    if (p[pos] != 0xFF) {return false;}
    while (pos < n && p[pos] == 0xFF) {++pos;}  // fill bytes
    if (pos >= n) {return false;}
    const uint8_t marker = p[pos++];
    if (marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) {continue;}  // no length
    if (marker == 0xD9 || marker == 0xDA) {return false;}  // EOI / SOS before any SOF
    if (pos + 2 > n) {return false;}
    const uint32_t len = readBe16(p + pos);
    if (len < 2) {return false;}
    // SOF0..SOF15 except DHT (C4), JPG (C8), DAC (CC): [len][precision][height][width].
    if (marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 &&
      marker != 0xCC)
    {
      if (len < 7 || pos + 7 > n) {return false;}
      height = readBe16(p + pos + 3);
      width = readBe16(p + pos + 5);
      return width > 0 && height > 0;
    }
    pos += len;
  }
  return false;
}

bool sizeAt(const uint8_t * p, size_t n, uint32_t & width, uint32_t & height)
{
  return jpegSize(p, n, width, height) || pngSize(p, n, width, height);
}

}  // namespace

bool compressedImageSize(
  const std::vector<uint8_t> & data,
  uint32_t & width,
  uint32_t & height)
{
  if (sizeAt(data.data(), data.size(), width, height)) {return true;}
  if (data.size() > kCompressedDepthHeaderSize) {
    return sizeAt(data.data() + kCompressedDepthHeaderSize,
             data.size() - kCompressedDepthHeaderSize, width, height);
  }
  return false;
}

}  // namespace sobits_vla
