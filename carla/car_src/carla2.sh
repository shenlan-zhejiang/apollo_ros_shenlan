#!/bin/zsh

# roslaunch carla_ros_bridge run_car_sim.launch & sleep 5

roslaunch carla_ros_bridge run_car_swarm_sim.launch synchronous_mode:=true & sleep 2

wait;
