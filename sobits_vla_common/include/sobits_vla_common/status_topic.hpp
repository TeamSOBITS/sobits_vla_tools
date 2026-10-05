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

#ifndef SOBITS_VLA_COMMON__STATUS_TOPIC_HPP_
#define SOBITS_VLA_COMMON__STATUS_TOPIC_HPP_

#include <cstdint>
#include <string>

namespace sobits_vla
{

// Empty when the service name does not end in "/command" (nothing to follow).
inline std::string statusTopicFromService(const std::string & service)
{
  const std::string suffix = "/command";
  if (service.size() <= suffix.size() ||
    service.compare(service.size() - suffix.size(), suffix.size(), suffix) != 0)
  {
    return "";
  }
  return service.substr(0, service.size() - suffix.size()) + "/status";
}

// Numbers mirror VlaCommand::Response::STATE_*; kept literal to stay ROS-free.
inline const char * vlaStateName(uint8_t state)
{
  switch (state) {
    case 0: return "STOPPED";
    case 1: return "RECORDING";
    case 2: return "PAUSED";
    case 3: return "PLAYING";
    case 4: return "ERROR";
    case 5: return "RESETTING";
    default: return "UNKNOWN";
  }
}

}  // namespace sobits_vla

#endif  // SOBITS_VLA_COMMON__STATUS_TOPIC_HPP_
