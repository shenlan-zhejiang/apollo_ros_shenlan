#!/bin/bash
cd ~/apollo_ros_bridge/car_src;
source devel/setup.bash;
roslaunch traj_planner swarm_apollo.launch 2> /dev/null;
