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

#ifndef WARNING_LAMP_MANAGER__WARNING_LAMP_MANAGER_HPP_
#define WARNING_LAMP_MANAGER__WARNING_LAMP_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"

namespace warning_lamp_manager
{
class WarningLampManager : public rclcpp::Node
{
public:
  explicit WarningLampManager(const rclcpp::NodeOptions & options);
  ~WarningLampManager();

private:
  #define ACTIVE_POLARITY (false)

  // Publisher
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_warning_lamp_emergency_;
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_warning_lamp_warning_;

  // Subscriber
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateMachine>::SharedPtr sub_state_;

  bool active_polarity_;
  bool use_overridable_vehicle_;

  void callbackStateMessage(
    const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg);
  void controlLampEmergency(const bool value);
  void controlLampWarning(const bool value);
  void warningLampManager(const uint16_t service_layer_state, const uint8_t control_layer_state);
};

}  // namespace warning_lamp_manager
#endif  // WARNING_LAMP_MANAGER__WARNING_LAMP_MANAGER_HPP_
