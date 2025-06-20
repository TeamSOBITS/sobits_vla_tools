#include <sensor_msgs/msg/joy.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <sobits_interfaces/action/vla_record_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sobits_vla
{

class GamepadClient : public rclcpp::Node
{
public:
  explicit GamepadClient(const rclcpp::NodeOptions & options);
  ~GamepadClient();
private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void sendGoal(const uint8_t & command);
  void goalResponseCallback(
    rclcpp_action::ClientGoalHandle<sobits_interfaces::action::VlaRecordState>::SharedPtr goal_handle);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<sobits_interfaces::action::VlaRecordState>::SharedPtr,
    const std::shared_ptr<const sobits_interfaces::action::VlaRecordState::Feedback> feedback);
  void resultCallback(
    const rclcpp_action::ClientGoalHandle<sobits_interfaces::action::VlaRecordState>::WrappedResult & result);

  rclcpp_action::Client<sobits_interfaces::action::VlaRecordState>::SendGoalOptions goal_options_;
  rclcpp_action::Client<sobits_interfaces::action::VlaRecordState>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;

  uint8_t current_state_;
  uint8_t previous_state_;

  std::string gamepad_name_;

  uint8_t record_button_;
  uint8_t pause_button_;
  uint8_t resume_button_;
  uint8_t save_button_;
  uint8_t delete_button_;

};

} // namespace sobits_vla

RCLCPP_COMPONENTS_REGISTER_NODE(sobits_vla::GamepadClient)
