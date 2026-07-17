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

#ifndef SOBITS_VLA_COMMON__GAMEPAD_CLT_HPP_
#define SOBITS_VLA_COMMON__GAMEPAD_CLT_HPP_

#include <string>

#include <sensor_msgs/msg/joy.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sobits_interfaces/srv/vla_command.hpp>

#include <rclcpp/rclcpp.hpp>

namespace sobits_vla
{

class GamepadClient : public rclcpp::Node
{
public:
  explicit GamepadClient(const rclcpp::NodeOptions & options);
  ~GamepadClient();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void callService(const uint8_t & command);
  void timerCallback();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Client<sobits_interfaces::srv::VlaCommand>::SharedPtr service_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;

  uint8_t current_state_;
  uint8_t previous_state_;

  std::string gamepad_name_;
  std::string command_service_name_;

  int record_button_;
  int pause_button_;
  int save_button_;
  int delete_button_;

  double button_cooldown_duration_;
  rclcpp::Time last_button_press_time_;
};

}  // namespace sobits_vla

#endif  // SOBITS_VLA_COMMON__GAMEPAD_CLT_HPP_
