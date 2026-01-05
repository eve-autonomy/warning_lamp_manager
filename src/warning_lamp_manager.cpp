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

#include "warning_lamp_manager/warning_lamp_manager.hpp"

namespace warning_lamp_manager
{

namespace
{
const char * getServiceLayerStateName(uint16_t state)
{
  switch (state) {
    case autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED:
      return "STATE_UNDEFINED";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE:
      return "STATE_CHECK_NODE_ALIVE";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP:
      return "STATE_EMERGENCY_STOP";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE:
      return "STATE_INFORM_ENGAGE";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART:
      return "STATE_INFORM_RESTART";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE:
      return "STATE_DURING_RECEIVE_ROUTE";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION:
      return "STATE_WAITING_ENGAGE_INSTRUCTION";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION:
      return "STATE_WAITING_CALL_PERMISSION";
    case autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL:
      return "STATE_ARRIVED_GOAL";
    default:
      return "DEFAULT";
  }
}
}  // namespace

WarningLampManager::WarningLampManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("warning_lamp_manager", options)
{
  use_overridable_vehicle_ = this->declare_parameter<bool>("use_overridable_vehicle", true);

  sub_initilization_state_ =
    this->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
      "/api/localization/initialization_state", rclcpp::QoS{3}.transient_local(),
      std::bind(
        &WarningLampManager::callbackAutowareInitializationMessage, this, std::placeholders::_1));

  sub_routing_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackRoutingStateMessage, this, std::placeholders::_1));

  sub_operation_mode_state_ =
    this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "/api/operation_mode/state", rclcpp::QoS{3}.transient_local(),
      std::bind(
        &WarningLampManager::callbackOperationModeStateMessage, this, std::placeholders::_1));

  // vehicle_status
  sub_calls_vehicle_state_ = this->create_subscription<go_interface_msgs::msg::VehicleStatus>(
    "api_vehicle_status", rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackVehicleStateMessage, this, std::placeholders::_1));

  sub_delivery_reservation_state_ =
    this->create_subscription<autoware_state_machine_msgs::msg::StateLock>(
      "/go_interface/lock_state", rclcpp::QoS{3}.transient_local(),
      std::bind(
        &WarningLampManager::callbackDeliveryReservationMessage, this, std::placeholders::_1));

  sub_engage_process_state_ = this->create_subscription<eve_cmd_gate_msgs::msg::EngageRequestState>(
    "/eve_cmd_gate/engage_request_state", rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackEngageProcessMessage, this, std::placeholders::_1));

  sub_hazard_status_ = this->create_subscription<autoware_system_msgs::msg::HazardStatusStamped>(
    "/system/emergency/hazard_status", rclcpp::QoS{1},
    std::bind(&WarningLampManager::callbackHazardStatusMessage, this, std::placeholders::_1));

  sub_motion_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::MotionState>(
    "/api/motion/state", rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackMotionStateMessage, this, std::placeholders::_1));

  pub_warning_lamp_emergency_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "lamp_emergency_out", rclcpp::QoS{3}.transient_local());
  pub_warning_lamp_warning_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "lamp_warning_out", rclcpp::QoS{3}.transient_local());

  active_polarity_ = ACTIVE_POLARITY;
  em_holding_ = false;
  motion_state_ = autoware_adapi_v1_msgs::msg::MotionState::UNKNOWN;
  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  initilization_state_ = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNKNOWN;
  routing_state_ = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
  delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  operation_state_.is_autoware_control_enabled = false;
  operation_state_.is_in_transition = false;
  operation_state_.is_stop_mode_available = false;
  operation_state_.is_autonomous_mode_available = false;
  operation_state_.is_local_mode_available = false;
  operation_state_.is_remote_mode_available = false;

  controlLampWarning(true);
  controlLampEmergency(true);
}

WarningLampManager::~WarningLampManager()
{
  controlLampWarning(false);
  controlLampEmergency(false);
}

void WarningLampManager::callbackAutowareInitializationMessage(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackAutowareInitializationMessage]autoware_state: %u", msg->state);

  initilization_state_ = msg->state;

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackRoutingStateMessage(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackRoutingStateMessage]routing_state: %u", msg->state);

  routing_state_ = msg->state;

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackOperationModeStateMessage(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  operation_state_ = *msg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackOperationModeStateMessage]operation mode: %u", msg->mode);

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackVehicleStateMessage(
  const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  flag_calls_vehicle_voice_ = msg->voice_flg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackOperationModeStateMessage]vheicle voice: %u", msg->voice_flg);

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackDeliveryReservationMessage(
  const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackDeliveryReservationMessage]"
    "StateLock: %u",
    msg->state);

  delivery_reservation_state_ = msg->state;
  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackEngageProcessMessage(
  const eve_cmd_gate_msgs::msg::EngageRequestState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackEngageProcessMessage]engage_request_state: %u, %u",
    msg->is_engage_requesting, msg->is_engage_accepted);

  is_engage_requesting_ = msg->is_engage_requesting;
  is_engage_accepted_ = msg->is_engage_accepted;
  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackHazardStatusMessage(
  const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  em_holding_ = msg->status.emergency_holding;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackHazardStatusMessage]emergency_holding: %s",
    em_holding_ ? "true" : "false");

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackMotionStateMessage(
  const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg)
{
  motion_state_ = msg->state;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::callbackMotionStateMessage]motion_state: %u", motion_state_);

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
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
      if (
        use_overridable_vehicle_ ||
        (control_layer_state == autoware_state_machine_msgs::msg::StateMachine::MANUAL)) {
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

bool WarningLampManager::isAutowareStateOfWaitingForRoute(void)
{
  return initilization_state_ ==
           autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZED &&
         routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::UNSET;
}

bool WarningLampManager::isAutowareStateOfPlanning(void)
{
  return routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::SET &&
         operation_state_.is_autonomous_mode_available;
}

bool WarningLampManager::isAutowareStateOfDriving(void)
{
  return operation_state_.mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS &&
         motion_state_ == autoware_adapi_v1_msgs::msg::MotionState::MOVING;
}

bool WarningLampManager::isAutowareStateOfArrivedGoal(void)
{
  return routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
}

bool WarningLampManager::checkStateInformEngage(void)
{
  return operation_state_.mode == autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS &&
         operation_state_.is_autoware_control_enabled &&
         motion_state_ == autoware_adapi_v1_msgs::msg::MotionState::STARTING;
}

bool WarningLampManager::checkStateInformRestart(void)
{
  return is_engage_requesting_ && !is_engage_accepted_;
}

bool WarningLampManager::checkState4DuringReceiveRoute(void)
{
  return isAutowareStateOfWaitingForRoute();
}

bool WarningLampManager::checkStateWaitingEngageInstruction(void)
{
  return operation_state_.mode == autoware_adapi_v1_msgs::msg::OperationModeState::STOP &&
         routing_state_ == autoware_adapi_v1_msgs::msg::RouteState::SET;
}

bool WarningLampManager::checkStateWaitingCallPermission(void)
{
  return isAutowareStateOfDriving() && flag_calls_vehicle_voice_ &&
         delivery_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_ON;
}

bool WarningLampManager::checkState4Arrived(void)
{
  return isAutowareStateOfArrivedGoal();
}

void WarningLampManager::changeState(void)
{
  if (service_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED) {
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE;
  } else if (em_holding_) {
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;
  } else if (checkStateInformEngage()) {
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE;
  } else if (checkStateInformRestart()) {
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART;
  } else if (checkState4DuringReceiveRoute()) {
    service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;
  } else if (checkStateWaitingEngageInstruction()) {
    service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;
  } else if (checkStateWaitingCallPermission()) {
    service_layer_state_ =
      autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION;
  } else if (checkState4Arrived()) {
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL;
  } else {
    service_layer_state_ = 0xFFFF;
  }

  if (operation_state_.is_autoware_control_enabled == false) {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  } else {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::AUTO;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1.0,
    "[WarningLampManager::changeState] service_layer_state: %s, control_layer_state: %s, "
    "operation_mode: %u, motion_state: %u, is_autoware_control_enabled: %s, em_holding: %s",
    getServiceLayerStateName(service_layer_state_),
    control_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::MANUAL ? "MANUAL"
                                                                                   : "AUTO",
    operation_state_.mode, motion_state_,
    operation_state_.is_autoware_control_enabled ? "true" : "false",
    em_holding_ ? "true" : "false");
}

}  // namespace warning_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(warning_lamp_manager::WarningLampManager)
