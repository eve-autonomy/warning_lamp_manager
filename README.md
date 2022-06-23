# Warning lamp manager

## Overview
This node controls warning and emergency lamps through `/dio_ros_driver` to alert pedestrians and vehicle drivers.

Lighting patterns are defined as follows.
|Name                   | Mode |Description|
|:----------------------|:---|:----------|
| Warning lamp | Always ON | - Autonomous driving vehicle is in driving mode to alert surrounding pedestrians. <br> - On manual omde. <br> - During system is in shutting down process to prevent not to remove external data storage. |
| " | One time ON | - When autonomous driving system boots up, system checks this lamp. |
| " | OFF | - Other cases |
| Emergency lamp | Always ON | - When the operation is stopped due to a system error |
| " | One time ON | - When the Autoware receives a engage topic. <br> - When autonomous driving system boots up, system checks this lamp. |
| " | OFF | - Other cases |

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
|use_overridable_vehicle|If autonomous driving vehicle has a function of override, set this parameter as `TRUE`. Otherwise set as `FALSE`.|
