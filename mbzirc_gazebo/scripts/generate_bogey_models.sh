#!/bin/bash

BOGEY_COUNT=5

if [ "$CATKIN_WS" == "" ]
then
  echo "You need to set the CATKIN_WS variable for this script."
  exit
fi

MODEL_DIR="$CATKIN_WS/src/mbzirc_gazebo/models/bogey"
MODEL_SDF_ERB="$MODEL_DIR/model.sdf.erb"
CONFIG_DIR="$MODEL_DIR/config"
CONFIG_MODEL_ERB="$CONFIG_DIR/px4_parameters.erb"

UDP_PORT_OFFSET=100
UDP_PORT_MAVROS_NODE=14540
UDP_PORT_MAVLINK_MAIN=14556
UDP_PORT_SIMULATOR_AUTOPILOT=14557
UDP_PORT_SIMULATOR_MAVLINK=14560

echo 'Generating Bogey models and PX4 SITL configurations'
for ((i = 0; i < BOGEY_COUNT; i++))
do
  MODEL_TAG=$(($i))
  MODEL_SDF="$MODEL_DIR/model$MODEL_TAG.sdf"
  MODEL_CONFIG="$CONFIG_DIR/px4_parameters$MODEL_TAG"

  erb \
    model_tag="$MODEL_TAG" \
    simulator_mavlink_udp_port=$UDP_PORT_SIMULATOR_MAVLINK \
    "$MODEL_SDF_ERB" > "$MODEL_SDF"
  erb \
    mavros_node_udp_port=$UDP_PORT_MAVROS_NODE \
    mavlink_main_udp_port=$UDP_PORT_MAVLINK_MAIN \
    simulator_autopilot_udp_port=$UDP_PORT_SIMULATOR_AUTOPILOT \
    simulator_mavlink_udp_port=$UDP_PORT_SIMULATOR_MAVLINK \
    "$CONFIG_MODEL_ERB" > "$MODEL_CONFIG"

  UDP_PORT_MAVROS_NODE=$((UDP_PORT_MAVROS_NODE + UDP_PORT_OFFSET))
  UDP_PORT_MAVLINK_MAIN=$((UDP_PORT_MAVLINK_MAIN + UDP_PORT_OFFSET))
  UDP_PORT_SIMULATOR_AUTOPILOT=$((UDP_PORT_SIMULATOR_AUTOPILOT + UDP_PORT_OFFSET))
  UDP_PORT_SIMULATOR_MAVLINK=$((UDP_PORT_SIMULATOR_MAVLINK + UDP_PORT_OFFSET))

  echo -ne '.'
done

echo -ne '\n'
exit 0
