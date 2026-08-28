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

#include "sobits_vla_common/gamepad_clt.hpp"

namespace sobits_vla
{

GamepadClient::GamepadClient(const rclcpp::NodeOptions & options)
: Node("gamepad_client", options),
  last_button_press_time_(0, 0, RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "Initializing GamepadClient Node...");

  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));

  // Create subscriber for Joy messages
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", qos_profile,
      std::bind(&GamepadClient::joyCallback, this, std::placeholders::_1));

  // Set values from parameters in the "gamepad" namespace.
  this->declare_parameter<std::string>(
    "gamepad.command_service", "vla_rosbag_collection/command");
  this->declare_parameter<std::string>("gamepad.controller", "dualshock4");
  this->declare_parameter<double>("gamepad.button_cooldown_duration", 0.5);
  // Service name selects the stage (deploy: sobits_vla_deploy/command,
  // collection: vla_rosbag_collection/command); matched by substring below.
  this->declare_parameter<std::string>("gamepad.deploy_service_match", "deploy");

  command_service_name_ = this->get_parameter("gamepad.command_service").as_string();
  const std::string deploy_match =
    this->get_parameter("gamepad.deploy_service_match").as_string();
  deploy_mode_ = !deploy_match.empty() &&
    command_service_name_.find(deploy_match) != std::string::npos;

  gamepad_name_ = this->get_parameter("gamepad.controller").as_string();
  button_cooldown_duration_ = this->get_parameter("gamepad.button_cooldown_duration").as_double();

  std::string base = std::string("gamepad.") + gamepad_name_ + ".button_mapping." +
    (deploy_mode_ ? "deploy." : "collection.");
  this->declare_parameter<int>(base + "record", -1);
  this->declare_parameter<int>(base + "pause", -1);
  this->declare_parameter<int>(base + "save", -1);
  this->declare_parameter<int>(base + "delete", -1);
  this->declare_parameter<int>(base + "play", -1);
  this->declare_parameter<int>(base + "reset", -1);

  record_button_ = this->get_parameter(base + "record").as_int();
  pause_button_ = this->get_parameter(base + "pause").as_int();
  save_button_ = this->get_parameter(base + "save").as_int();
  delete_button_ = this->get_parameter(base + "delete").as_int();

  // For deploy mode (play/stop toggle + reset)
  play_button_ = this->get_parameter(base + "play").as_int();
  reset_button_ = this->get_parameter(base + "reset").as_int();

  // Log params
  RCLCPP_INFO(this->get_logger(), "Command Service Name: %s", command_service_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Stage: %s", deploy_mode_ ? "deploy" : "collection");
  RCLCPP_INFO(this->get_logger(), "Gamepad name: %s", gamepad_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Record button: %d", record_button_);
  RCLCPP_INFO(this->get_logger(), "Pause button: %d", pause_button_);
  RCLCPP_INFO(this->get_logger(), "Save button: %d", save_button_);
  RCLCPP_INFO(this->get_logger(), "Delete button: %d", delete_button_);
  RCLCPP_INFO(this->get_logger(), "Reset button: %d", reset_button_);
  RCLCPP_INFO(this->get_logger(), "Cooldown duration: %.2f s", button_cooldown_duration_);

  // Create service client for VlaCommand
  service_client_ = this->create_client<sobits_interfaces::srv::VlaCommand>(command_service_name_);

  // Create wall timer to periodically check the joy messages
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&GamepadClient::timerCallback, this));

  // Init values
  current_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;
  previous_state_ = sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED;

  RCLCPP_INFO(this->get_logger(), "GamepadClient initialized successfully.");
}

GamepadClient::~GamepadClient()
{
  if (joy_subscriber_) {
    joy_subscriber_.reset();
  }
  if (service_client_) {
    service_client_.reset();
  }
  RCLCPP_INFO(this->get_logger(), "GamepadClient destructor called");
}

void GamepadClient::timerCallback()
{
  // Check if we have received a Joy message
  if (!last_joy_msg_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No Joy message received yet");
    return;
  }

  // Check if the service server is available
  if (!service_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Service server not available, cannot process commands");
    return;
  }

  // Terminate node if the current state is STATE_ERROR
  if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_ERROR) {
    RCLCPP_ERROR(this->get_logger(), "Current state is STATE_ERROR, cannot process commands");
    return;
  }

  // Check the last joy message for button presses
  rclcpp::Time now = this->now();
  if ((now - last_button_press_time_).seconds() < button_cooldown_duration_) {
    return;
  }

  bool button_pressed = false;

  // Negative index → axes, non-negative → buttons
  auto pressed = [&](int idx) -> bool {
      if (idx < 0) {
        const size_t ax = static_cast<size_t>(std::abs(idx));
        return ax < last_joy_msg_->axes.size() && last_joy_msg_->axes[ax] > 0.5f;
      }
      return static_cast<int>(idx) < static_cast<int>(last_joy_msg_->buttons.size()) &&
             last_joy_msg_->buttons[idx] != 0;
    };

  if (deploy_mode_) {
    // Reset: force a fresh episode — STOP in any state (the deploy node
    // aborts a running episode, resets model state and re-poses the robot).
    if (reset_button_ != -1 && pressed(reset_button_)) {
      callService(sobits_interfaces::srv::VlaCommand::Request::STOP);
      button_pressed = true;
    }
    // Play button: PLAY when stopped, STOP while playing.
    if (!button_pressed && play_button_ != -1 && pressed(play_button_)) {
      if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_PLAYING) {
        callService(sobits_interfaces::srv::VlaCommand::Request::STOP);
      } else {
        callService(sobits_interfaces::srv::VlaCommand::Request::PLAY);
      }
      button_pressed = true;
    }
    if (button_pressed) {
      last_button_press_time_ = now;
    }
    return;
  }

  // Toggle Record/Pause/Resume
  if (record_button_ != -1 && pressed(record_button_)) {
    if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED) {
      callService(sobits_interfaces::srv::VlaCommand::Request::RECORD);
      button_pressed = true;
    } else if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING) {
      callService(sobits_interfaces::srv::VlaCommand::Request::PAUSE);
      button_pressed = true;
    } else if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED) {
      callService(sobits_interfaces::srv::VlaCommand::Request::RESUME);
      button_pressed = true;
    }
  }

  // Toggle Pause/Resume separately (if mapped to a different button)
  if (!button_pressed && pause_button_ != -1 && pressed(pause_button_)) {
    if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING) {
      callService(sobits_interfaces::srv::VlaCommand::Request::PAUSE);
      button_pressed = true;
    } else if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED) {
      callService(sobits_interfaces::srv::VlaCommand::Request::RESUME);
      button_pressed = true;
    }
  }

  // Reset the scene between episodes. Only while stopped: a teleport during
  // recording would land in the bag as a discontinuity.
  if (!button_pressed && reset_button_ != -1 && pressed(reset_button_)) {
    if (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED) {
      callService(sobits_interfaces::srv::VlaCommand::Request::RESET);
      button_pressed = true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Ignoring RESET while recording or paused.");
    }
  }

  // Save / Delete toggle (same button)
  if (!button_pressed && save_button_ != -1 && pressed(save_button_)) {
    const bool is_rec =
      (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_RECORDING);
    const bool is_paused =
      (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_PAUSED);
    const bool is_stopped =
      (current_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED);
    const bool was_stopped =
      (previous_state_ == sobits_interfaces::srv::VlaCommand::Response::STATE_STOPPED);

    if (is_rec || is_paused) {
      callService(sobits_interfaces::srv::VlaCommand::Request::SAVE);
      button_pressed = true;
    } else if (is_stopped && !was_stopped) {
      callService(sobits_interfaces::srv::VlaCommand::Request::DELETE);
      button_pressed = true;
    }
  }

  if (button_pressed) {
    last_button_press_time_ = now;
  }
}

void GamepadClient::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_msg_ = msg;
}

void GamepadClient::callService(const uint8_t & command)
{
  auto request = std::make_shared<sobits_interfaces::srv::VlaCommand::Request>();
  request->command = command;

  RCLCPP_INFO(this->get_logger(), "Sending command request: %s",
      command == sobits_interfaces::srv::VlaCommand::Request::RECORD ? "RECORD" :
      command == sobits_interfaces::srv::VlaCommand::Request::PAUSE ? "PAUSE" :
      command == sobits_interfaces::srv::VlaCommand::Request::RESUME ? "RESUME" :
      command == sobits_interfaces::srv::VlaCommand::Request::SAVE ? "SAVE" :
      command == sobits_interfaces::srv::VlaCommand::Request::DELETE ? "DELETE" :
      command == sobits_interfaces::srv::VlaCommand::Request::PLAY ? "PLAY" :
      command == sobits_interfaces::srv::VlaCommand::Request::STOP ? "STOP" : "UNKNOWN");

  // Call the service asynchronously
  auto result_future = service_client_->async_send_request(
      request,
    [this](rclcpp::Client<sobits_interfaces::srv::VlaCommand>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", response->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", response->message.c_str());
      }
      // Status authoritative on both paths: a failed SAVE still stops the server when
      // it discards a too-short episode. Ignoring it strands the client in PAUSED.
      if (response->status != this->current_state_) {
        this->previous_state_ = this->current_state_;
        this->current_state_ = response->status;
      }
    });
}

}  // namespace sobits_vla

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sobits_vla::GamepadClient)
