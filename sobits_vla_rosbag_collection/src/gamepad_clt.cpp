#include "sobits_vla_rosbag_collection/gamepad_clt.hpp"

namespace sobits_vla
{

GamepadClient::GamepadClient(const rclcpp::NodeOptions & options)
: Node("gamepad_client", options)
{
  // TODO: Configure QoS settings
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  // qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  // qos.durability(rclcpp::DurabilityPolicy::Volatile);
  // qos.history(rclcpp::HistoryPolicy::KEEP_LAST);

  // Create subscriber for Joy messages
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", qos_profile,
      std::bind(&GamepadClient::joyCallback, this, std::placeholders::_1));

  // Create action client for VlaRecordState
  action_client_ = rclcpp_action::create_client<sobits_interfaces::action::VlaRecordState>(
      this, "vla_record_state");
  // Wait for the action server to be available
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return;
  }
  // Set up goal options
  goal_options_.goal_response_callback = std::bind(
      &GamepadClient::goalResponseCallback, this, std::placeholders::_1);
  goal_options_.feedback_callback = std::bind(
      &GamepadClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  goal_options_.result_callback = std::bind(
      &GamepadClient::resultCallback, this, std::placeholders::_1);

  // Set values from parameters
  this->declare_parameter<std::string>("gamepad_config.gamepad_name", "dualshock4");
  gamepad_name_ = this->get_parameter("gamepad_name").as_string();
  // Terminate if gamepad name value is "keyboard"
  if (gamepad_name_ == "keyboard") {
    RCLCPP_ERROR(this->get_logger(), "Gamepad name cannot be 'keyboard'. Please set a valid gamepad name.");
    rclcpp::shutdown();
    return;
  } 

  this->declare_parameter<uint8_t>("gamepad_config." + gamepad_name_ + ".button_mapping.record", 0);
  this->declare_parameter<uint8_t>("gamepad_config." + gamepad_name_ + ".button_mapping.pause", 0);
  this->declare_parameter<uint8_t>("gamepad_config." + gamepad_name_ + ".button_mapping.resume", 0);
  this->declare_parameter<uint8_t>("gamepad_config." + gamepad_name_ + ".button_mapping.save", 0);
  this->declare_parameter<uint8_t>("gamepad_config." + gamepad_name_ + ".button_mapping.delete", 0);
  record_button_ = this->get_parameter("gamepad_config." + gamepad_name_ + ".button_mapping.record").as_int();
  pause_button_  = this->get_parameter("gamepad_config." + gamepad_name_ + ".button_mapping.pause").as_int();
  resume_button_ = this->get_parameter("gamepad_config." + gamepad_name_ + ".button_mapping.resume").as_int();
  save_button_   = this->get_parameter("gamepad_config." + gamepad_name_ + ".button_mapping.save").as_int();
  delete_button_ = this->get_parameter("gamepad_config." + gamepad_name_ + ".button_mapping.delete").as_int();

  // Init values
  current_state_  = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
  previous_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;

  RCLCPP_INFO(this->get_logger(), "GamepadClient initialized with gamepad: %s", gamepad_name_.c_str());
}

GamepadClient::~GamepadClient()
{
  // Clean up resources
  if (joy_subscriber_) {
    joy_subscriber_.reset();
  }
  if (action_client_) {
    action_client_->async_cancel_all_goals();
    action_client_.reset();
  }
  RCLCPP_INFO(this->get_logger(), "GamepadClient destructor called");
}

void GamepadClient::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Check button states
  if (msg->buttons[record_button_] && current_state_ != sobits_interfaces::action::VlaRecordState_Result::RECORDING) {
    sendGoal(sobits_interfaces::action::VlaRecordState_Goal::RECORD);
  } else if (msg->buttons[pause_button_] && current_state_ == sobits_interfaces::action::VlaRecordState_Result::RECORDING) {
    sendGoal(sobits_interfaces::action::VlaRecordState_Goal::PAUSE);
  } else if (msg->buttons[resume_button_] && current_state_ == sobits_interfaces::action::VlaRecordState_Result::PAUSED) {
    sendGoal(sobits_interfaces::action::VlaRecordState_Goal::RESUME);
  } else if (msg->buttons[save_button_] && current_state_ != sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
    sendGoal(sobits_interfaces::action::VlaRecordState_Goal::SAVE);
  } else if (msg->buttons[delete_button_]) {
    sendGoal(sobits_interfaces::action::VlaRecordState_Goal::DELETE);
  }
}

void GamepadClient::goalResponseCallback(
  rclcpp_action::ClientGoalHandle<sobits_interfaces::action::VlaRecordState>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server");
}

void GamepadClient::feedbackCallback(
  rclcpp_action::ClientGoalHandle<sobits_interfaces::action::VlaRecordState>::SharedPtr,
  const std::shared_ptr<const sobits_interfaces::action::VlaRecordState::Feedback> feedback)
{
  // Process feedback if needed
  RCLCPP_DEBUG(this->get_logger(), "Received feedback: %s", feedback->status.c_str());
}

void GamepadClient::resultCallback(
  const rclcpp_action::ClientGoalHandle<sobits_interfaces::action::VlaRecordState>::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Action failed with code: %d", static_cast<int>(result.code));
    return;
  }

  // Process the result
  RCLCPP_INFO(this->get_logger(), "Action succeeded with result: %s", 
      result.result->status == sobits_interfaces::action::VlaRecordState_Result::RECORDING ? "RECORDING" :
      result.result->status == sobits_interfaces::action::VlaRecordState_Result::PAUSED ? "PAUSED" :
      result.result->status == sobits_interfaces::action::VlaRecordState_Result::STOPPED ? "STOPPED" :
      "UNKNOWN");

  // Update current state based on the command
  previous_state_ = current_state_;
  if (result.result->status == sobits_interfaces::action::VlaRecordState_Result::RECORDING) {
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::RECORDING;
  } else if (result.result->status == sobits_interfaces::action::VlaRecordState_Result::PAUSED) {
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::PAUSED;
  } else if (result.result->status == sobits_interfaces::action::VlaRecordState_Result::STOPPED) {
    current_state_ = sobits_interfaces::action::VlaRecordState_Result::STOPPED;
  }
}

void GamepadClient::sendGoal(const uint8_t & command)
{
  // Create a new goal
  auto goal_msg = sobits_interfaces::action::VlaRecordState::Goal();
  goal_msg.command = command;

  // Send the goal to the action server
  RCLCPP_INFO(this->get_logger(), "Send goal: %s",
      command == sobits_interfaces::action::VlaRecordState_Goal::RECORD ? "RECORD" :
      command == sobits_interfaces::action::VlaRecordState_Goal::PAUSE ? "PAUSE" :
      command == sobits_interfaces::action::VlaRecordState_Goal::RESUME ? "RESUME" :
      command == sobits_interfaces::action::VlaRecordState_Goal::SAVE ? "SAVE" :
      command == sobits_interfaces::action::VlaRecordState_Goal::DELETE ? "DELETE" : "UNKNOWN");
  this->action_client_->async_send_goal(goal_msg, goal_options_);

}

} // namespace sobits_vla
