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
#include "dio_ros_driver/msg/dio_port.hpp"
#include "go_interface_msgs/msg/vehicle_status.hpp"
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/diag_graph_status.hpp>
#include <autoware_adapi_v1_msgs/msg/diag_graph_struct.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>


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
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>::SharedPtr sub_initilization_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr sub_routing_state_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::DiagGraphStatus>::SharedPtr sub_daignostics_status_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::DiagGraphStruct>::SharedPtr sub_daignostics_struct_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<go_interface_msgs::msg::VehicleStatus>::SharedPtr sub_calls_vehicle_state_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateLock>::SharedPtr sub_delivery_reservation_state_;
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateMachine>::SharedPtr sub_engage_process_state_;
  
  std::shared_mutex mtx_;
  bool is_engage_requesting_;
  bool is_engage_accepted_;

  bool active_polarity_;
  bool use_overridable_vehicle_;
  uint16_t service_layer_state_;
  uint16_t control_layer_state_;
  uint16_t initilization_state_;
  uint16_t routing_state_;
  uint16_t delivery_reservation_state_;  
  std::optional<uint16_t> em_holding_indices_;
  bool em_holding_;
  autoware_adapi_v1_msgs::msg::OperationModeState operation_state_;
  bool flag_calls_vehicle_voice_;

  void callbackStateMessage(
    const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg);
  void callbackAutowareInitializationMessage(
    const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg);
  void callbackRoutingStateMessage(
    const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void callbackDaignosticsStructMessage(
    const autoware_adapi_v1_msgs::msg::DiagGraphStruct::ConstSharedPtr msg);
  void callbackDaignosticsStateMessage(
    const autoware_adapi_v1_msgs::msg::DiagGraphStatus::ConstSharedPtr msg);
  void callbackOperationModeStateMessage(
    const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void callbackVehicleStateMessage(
    const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg);
  void callbackDeliveryReservationMessage(
    const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg);
  void callbackEngageProcessMessage(
    const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg);

  void controlLampEmergency(const bool value);
  void controlLampWarning(const bool value);
  void warningLampManager(const uint16_t service_layer_state, const uint8_t control_layer_state);
  void changeState(void);
  bool isAutowareStateOfWaitingForEngage(void);
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
  void setEngageProcess(bool request, bool accept);
  std::pair<bool, bool> getEngageProcess();
};

}  // namespace warning_lamp_manager
#endif  // WARNING_LAMP_MANAGER__WARNING_LAMP_MANAGER_HPP_
