#!/bin/bash
cd ~/apollo_ros_bridge/apollo2ros_mapping;
source devel/setup.bash;
rosparam set use_sim_time false;
rosrun mapping mapping;
