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

  // autoware_state
  sub_initilization_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::LocalizationInitializationState>(
    "/api/localization/initialization_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackAutowareInitializationMessage, this, std::placeholders::_1)
  );

  // routing wait state
  sub_routing_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackRoutingStateMessage, this, std::placeholders::_1)
  );

  // daignostics struct for EM Holding
  sub_daignostics_struct_ = this->create_subscription<autoware_adapi_v1_msgs::msg::DiagGraphStruct>(
    "/api/system/diagnostics/struct",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackDaignosticsStructMessage, this, std::placeholders::_1)
  );
  sub_daignostics_status_ = this->create_subscription<autoware_adapi_v1_msgs::msg::DiagGraphStatus>(
    "/api/system/diagnostics/status",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackDaignosticsStateMessage, this, std::placeholders::_1)
  );

  // OperationModeState
  sub_operation_mode_state_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackOperationModeStateMessage, this, std::placeholders::_1)
  );

  // vehicle_status
  sub_calls_vehicle_state_ = this->create_subscription<go_interface_msgs::msg::VehicleStatus>(
    "api_vehicle_status",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackVehicleStateMessage, this, std::placeholders::_1)
  );

  // delivery reservation state
  sub_delivery_reservation_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateLock>(
    "/go_interface/lock_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackDeliveryReservationMessage, this, std::placeholders::_1)
  );

  // TODO:eve_cmd_gateからengage状態をもらう、topic名とメッセージ型を反映
  // engage process state
  sub_engage_process_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateMachine>(
    "xxxxx/xxxx",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&WarningLampManager::callbackEngageProcessMessage, this, std::placeholders::_1)
  );

  pub_warning_lamp_emergency_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "lamp_emergency_out",
    rclcpp::QoS{3}.transient_local());
  pub_warning_lamp_warning_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "lamp_warning_out",
    rclcpp::QoS{3}.transient_local());

  active_polarity_ = ACTIVE_POLARITY;
  em_holding_indices_ = std::nullopt;
  service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED;
  control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  initilization_state_ = autoware_adapi_v1_msgs::msg::LocalizationInitializationState::UNKNOWN;
  routing_state_ = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;
  delivery_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  em_holding_ = false;
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

void WarningLampManager::callbackAutowareInitializationMessage(
  const autoware_adapi_v1_msgs::msg::LocalizationInitializationState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[WarningLampManager::callbackAutowareInitializationMessage]autoware_state: %u",
    msg->state);

  initilization_state_ = msg->state;

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackRoutingStateMessage(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[WarningLampManager::callbackRoutingStateMessage]routing_state: %u",
    msg->state);

  routing_state_ = msg->state;

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackDaignosticsStructMessage(
  const autoware_adapi_v1_msgs::msg::DiagGraphStruct::ConstSharedPtr msg)
{
  auto nodes = msg->nodes;

  for (uint16_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].path == "/autoware/modes/autonomous") {
      em_holding_indices_ = i;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1.0,
        "[WarningLampManager::callbackDaignosticsStructMessage]daignostics_graph /autoware/modes/autonomous index: %u", i);
      break;
    }
  }
}

void WarningLampManager::callbackDaignosticsStateMessage(
  const autoware_adapi_v1_msgs::msg::DiagGraphStatus::ConstSharedPtr msg)
{
  auto nodes = msg->nodes;
  if (em_holding_indices_ != std::nullopt) {
    // TODO:Ph3にて、levelをlatch_levelに変更
    if (nodes[em_holding_indices_.value()].level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      em_holding_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1.0,
        "[WarningLampManager::callbackDaignosticsStateMessage]/autoware/modes/autonomous latch_level: %u",
        nodes[em_holding_indices_.value()].level);// TODO:Ph3にて、levelをlatch_levelに変更
    } else {
      em_holding_ = false;
    }

    changeState();
    warningLampManager(service_layer_state_, control_layer_state_);
  }
}

void WarningLampManager::callbackOperationModeStateMessage(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  operation_state_ = *msg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[WarningLampManager::callbackOperationModeStateMessage]operation mode: %u",
      msg->mode);

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackVehicleStateMessage(
  const go_interface_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  flag_calls_vehicle_voice_ = msg->voice_flg;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[WarningLampManager::callbackOperationModeStateMessage]vheicle voice: %u",
      msg->voice_flg);

  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackDeliveryReservationMessage(
  const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[WarningLampManager::callbackDeliveryReservationMessage]"
    "StateLock: %u",
    msg->state);

  delivery_reservation_state_ = msg->state;
  changeState();
  warningLampManager(service_layer_state_, control_layer_state_);
}

void WarningLampManager::callbackEngageProcessMessage(
  const autoware_state_machine_msgs::msg::StateMachine::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[WarningLampManager::callbackEngageProcessMessage]"
    "request: %u, accept: %u",
    msg->service_layer_state, msg->control_layer_state);
//    msg->request, msg->accept);

  //setEngageProcess(msg->request, msg->accept);
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

void WarningLampManager::setEngageProcess(bool request, bool accept)
{
  /* In multi-threaded system, there is a possibility of simultaneous accesses from different
     threads, so exclusion control is performed. Set request to "true" when we want the audio to
     play when vehicle departs and restarts. Set accept to "true" when the audio playback is
     complete. If you want to suspend waiting for playback to complete for some reason, set both
     values to "false". */
  std::lock_guard<std::shared_mutex> lock(mtx_);
  is_engage_requesting_ = request;
  is_engage_accepted_ = accept;
}

std::pair<bool, bool> WarningLampManager::getEngageProcess()
{
  /* In multi-threaded system, there is a possibility of simultaneous accesses from different
     threads, so exclusion control is performed. Check both values to determine if the playback is
     played, interrupted, or completed. */
  std::pair<bool, bool> value;
  {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    value = std::make_pair(is_engage_requesting_, is_engage_accepted_);
  }
  return value;
}

bool WarningLampManager::isAutowareStateOfWaitingForRoute(void)
{
  if (initilization_state_ != autoware_adapi_v1_msgs::msg::LocalizationInitializationState::INITIALIZING) {
    return false;
  }
  if (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::UNSET) {
    return false;
  }

  return true;
}

bool WarningLampManager::isAutowareStateOfPlanning(void)
{
  if (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::SET) {
    return false;
  }

  return true;
}

bool WarningLampManager::isAutowareStateOfWaitingForEngage(void)
{
  if(operation_state_.mode != autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS) {
    return false;
  }
  if (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::SET) {
    return false;
  }

  return true;
}

bool WarningLampManager::isAutowareStateOfDriving(void)
{
  if(operation_state_.mode != autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS) {
    return false;
  }

  return true;
}

bool WarningLampManager::isAutowareStateOfArrivedGoal(void)
{
  if (routing_state_ != autoware_adapi_v1_msgs::msg::RouteState::ARRIVED) {
    return false;
  }

  // flag_arrived_state_machine_ = true;
  return true;
}


bool WarningLampManager::checkStateInformEngage(void)
{
  if (isAutowareStateOfDriving() != true){
    return false;
  }

  if (service_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION) {
    if (flag_calls_vehicle_voice_ == true || delivery_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_ON) {
      return false;
    }
  }

  auto [is_request, is_accept] = getEngageProcess();
  if (is_request == false || delivery_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION) {
      return false;
  }

  return true;
}

bool WarningLampManager::checkStateInformRestart(void)
{
  if (isAutowareStateOfDriving() != true){
    return false;
  }

  /* If restart is requested, voice guidance will be played. */
  auto [is_request, is_accept] = getEngageProcess();
  if (!is_request || is_accept) {  
    return false;
  }

  return true;
}

bool WarningLampManager::checkState4DuringReceiveRoute(void)
{
  if (isAutowareStateOfWaitingForRoute() != true &&
      isAutowareStateOfPlanning() != true){
    return false;
  }

  return true;
}

bool WarningLampManager::checkStateWaitingEngageInstruction(void)
{
  if(control_layer_state_ != autoware_state_machine_msgs::msg::StateMachine::MANUAL) {
    return false;
  }

  if (isAutowareStateOfDriving() != true){
    return false;
  }

  return true;
}

bool WarningLampManager::checkStateWaitingCallPermission(void)
{
  if (isAutowareStateOfDriving() != true){
    return false;
  }

  // delivery_reservation_state_ は追加トピックから取得予定
  if ((flag_calls_vehicle_voice_ != true) || 
      (delivery_reservation_state_ != autoware_state_machine_msgs::msg::StateLock::STATE_ON)) {
      return false;
  }

  return true;  
}

bool WarningLampManager::checkState4Arrived(void)
{
  if (isAutowareStateOfArrivedGoal() != true){
    return false;
  }

  return true;  
}

void WarningLampManager::changeState(void)
{
  if (service_layer_state_ == autoware_state_machine_msgs::msg::StateMachine::STATE_UNDEFINED) { 
    // STATE_CHECK_NODE_ALIVE
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_CHECK_NODE_ALIVE;

  } else if (em_holding_ == true) {
    //setEngageProcess(false, false);
    // STATE_EMERGENCY_STOP
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_EMERGENCY_STOP;

  } else if (checkStateInformEngage() == true) {
    // STATE_INFORM_ENGAGE
    //setEngageProcess(false, true);
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_ENGAGE;

  } else if (checkStateInformRestart() == true) {
    // STATE_INFORM_RESTART
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_INFORM_RESTART;

  } else if (checkState4DuringReceiveRoute() == true) {
    // STATE_DURING_RECEIVE_ROUTE
    //setEngageProcess(false, false);
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_DURING_RECEIVE_ROUTE;

  } else if (checkStateWaitingEngageInstruction() == true) {
    // STATE_WAITING_ENGAGE_INSTRUCTION
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_ENGAGE_INSTRUCTION;

  } else if (checkStateWaitingCallPermission() == true) {
    // STATE_WAITING_CALL_PERMISSION
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_WAITING_CALL_PERMISSION;

  } else if (checkState4Arrived() == true) {
    // STATE_ARRIVED_GOAL
    //setEngageProcess(false, false);    
    service_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::STATE_ARRIVED_GOAL;

  } else {
    // 上記以外
    service_layer_state_ = 0xFFFF;
  }

  if ((operation_state_.is_stop_mode_available == true) ||
      (operation_state_.is_local_mode_available == true)) {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::MANUAL;
  } else {
    control_layer_state_ = autoware_state_machine_msgs::msg::StateMachine::AUTO;
  }
}

}  // namespace warning_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(warning_lamp_manager::WarningLampManager)
