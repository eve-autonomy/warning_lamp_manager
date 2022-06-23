# Warning lamp manager

## Overview
This node controls warning and emergency lamps through `/dio_ros_driver` to alert pedestrians and vehicle drivers.

Lighting patterns are defined as follows.
- warning lamp
  - Lights up whenever the vehicle may move automatically to alert pedestrians around.
  - Always lit in manual mode. 
  - Lights up immediately after the system boots for daily inspection.
  - Always lit during the system shutdown process to alert the user not to accidentally remove the external storage medium.
  - Keeps going off in other cases.
- emergency lamp
  - Always lights up when the operation is stopped due to a system error.
  - Lights up the moment the vehicle starts.
  - Lights up immediately after the system boots for daily inspection.
  - Keeps going off in other cases.

## Input and Output
- input
  - from [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine/)
    - `/autoware_state_machine/state` : State of the system.
- output
  - to [dio_ros_driver](https://github.com/tier4/dio_ros_driver/)
    - `/dio/dout1` : GPIO output topic for emergency lamp. (this topic is remapping from lamp_emergency_out)
    - `/dio/dout2` : GPIO output topic for warning lamp. (this topic is remapping from lamp_warning_out)
## Node Graph
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/warning_lamp_manager/docs/node_graph.pu)

## Lanch arguments
|Name                   |Description|
|:----------------------|:----------|
|use_overridable_vehicle|If autonomous driving vehicle has a function of override, set this parameter as `TRUE`. Otherwise set as `FALSE`. This value overrides on a parameter with same name.|

## Paramater description
|Name                   |Description|
|:----------------------|:----------|
|use_overridable_vehicle|Whether the vehicle can transfer driving authority by the operator's brakes.|
