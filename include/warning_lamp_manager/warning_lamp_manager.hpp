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
#include "autoware_state_machine_msgs/msg/state_lock.hpp"
#include "autoware_state_machine_msgs/msg/state_machine.hpp"
#include "autoware_system_msgs/msg/hazard_status_stamped.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include "eve_cmd_gate_msgs/msg/engage_request_state.hpp"
#include "go_interface_msgs/msg/vehicle_status.hpp"
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>


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
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr sub_initilization_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_routing_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<go_interface_msgs::msg::VehicleStatus>::SharedPtr sub_calls_vehicle_state_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateLock>::SharedPtr sub_delivery_reservation_state_;
  rclcpp::Subscription<eve_cmd_gate_msgs::msg::EngageRequestState>::SharedPtr sub_engage_process_state_;
  rclcpp::Subscription<autoware_system_msgs::msg::HazardStatusStamped>::SharedPtr sub_hazard_status_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MotionState>::SharedPtr sub_motion_state_;

  uint16_t motion_state_;
  bool is_engage_requesting_;
  bool is_engage_accepted_;
  bool active_polarity_;
  bool use_overridable_vehicle_;
  bool em_holding_;
  uint16_t service_layer_state_;
  uint16_t control_layer_state_;
  uint16_t initilization_state_;
  uint16_t routing_state_;
  uint16_t delivery_reservation_state_;
  autoware_adapi_v1_msgs::msg::OperationModeState operation_state_;
  bool flag_calls_vehicle_voice_;

  void callbackAutowareInitializationMessage(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void callbackRoutingStateMessage(
    const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void callbackOperationModeStateMessage(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void callbackVehicleStateMessage(
    const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg);
  void callbackDeliveryReservationMessage(
    const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg);
  void callbackEngageProcessMessage(
    const eve_cmd_gate_msgs::msg::EngageRequestState::ConstSharedPtr msg);
  void callbackHazardStatusMessage(
    const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
  void callbackMotionStateMessage(
    const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg);

  void controlLampEmergency(const bool value);
  void controlLampWarning(const bool value);
  void warningLampManager(const uint16_t service_layer_state, const uint8_t control_layer_state);
  void changeState(void);
  bool isAutowareStateOfDriving(void);
  bool isAutowareStateOfWaitingForRoute(void);
  bool isAutowareStateOfPlanning(void);
  bool isAutowareStateOfArrivedGoal(void);
  bool checkStateInformEngage(void);
  bool checkStateInformRestart(void);
  bool checkState4DuringReceiveRoute(void);
  bool checkStateWaitingEngageInstruction(void);
  bool checkStateWaitingCallPermission(void);
  bool checkState4Arrived(void);
};

}  // namespace warning_lamp_manager
#endif  // WARNING_LAMP_MANAGER__WARNING_LAMP_MANAGER_HPP_
