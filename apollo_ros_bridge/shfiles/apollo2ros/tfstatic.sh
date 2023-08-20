#!/bin/bash

echo "child_frame_id: lidar
transform:
  x: 0 
  y: 0 
  z: 1.7 
rotation: 
  x: 0 
  y: 0 
  z: 0.7071 
  w: 0.7071"
  
rosrun tf static_transform_publisher 0 0 1.7 0 0 0.7071 0.7071 agent_0 lidar 0.01

