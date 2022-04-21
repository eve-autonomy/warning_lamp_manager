// Copyright 2020 eve autonomy inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#include <fstream>

#include "warning_lamp_manager/warning_lamp_manager.hpp"

namespace warning_lamp_manager
{

WarningLampManager::WarningLampManager(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("warning_lamp_manager", options)
{
  use_overridable_vehicle_ = this->declare_parameter<bool>(
    "use_overridable_vehicle", true);

  sub_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateMachine>(
    "autoware_state_machine/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackStateMessage, this, std::placeholders::_1)
  );
  pub_warning_lamp_emergency_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "lamp_emergency_out",
    rclcpp::QoS{3}.transient_local());
  pub_warning_lamp_warning_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "lamp_warning_out",
    rclcpp::QoS{3}.transient_local());

  active_polarity_ = ACTIVE_POLARITY;

  controlLampWarning(true);
  controlLampEmergency(true);
}

WarningLampManager::~WarningLampManager()
{
  controlLampWarning(false);
  controlLampEmergency(false);
}

void WarningLampManager::callbackStateMessage(
  const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[WarningLampManager::callbackStateMessage]"
    "service_layer_state: %u, control_layer_state: %u",
    msg->service_layer_state,
    msg->control_layer_state);

  warningLampManager(msg->service_layer_state, msg->control_layer_state);
}

void WarningLampManager::controlLampEmergency(const bool value)
{
  dio_ros_driver::msg::DIOPort msg;
  msg.value = active_polarity_ ? value : !value;
  pub_warning_lamp_emergency_->publish(msg);
}

void WarningLampManager::controlLampWarning(const bool value)
{
  dio_ros_driver::msg::DIOPort msg;
  msg.value = active_polarity_ ? value : !value;
  pub_warning_lamp_warning_->publish(msg);
}

void WarningLampManager::warningLampManager(
  const uint16_t service_layer_state, const uint8_t control_layer_state)
{
  switch (service_layer_state) {
    case autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE:
      controlLampWarning(true);
      controlLampEmergency(true);
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP:
      controlLampWarning(false);
      controlLampEmergency(true);
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART:
      if (control_layer_state == autoware_state_machine_msgs::msg::StateMachine::MANUAL) {
        controlLampWarning(true);
        controlLampEmergency(false);
      } else {
        controlLampWarning(true);
        controlLampEmergency(true);
      }
      break;

    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION:
    case autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL:
      if (use_overridable_vehicle_ ||
        (control_layer_state == autoware_state_machine_msgs::msg::StateMachine::MANUAL))
      {
        controlLampWarning(true);
        controlLampEmergency(false);
      } else {
        controlLampWarning(false);
        controlLampEmergency(false);
      }
      break;

    default:
      controlLampWarning(true);
      controlLampEmergency(false);
      break;
  }
}

}  // namespace warning_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(warning_lamp_manager::WarningLampManager)
