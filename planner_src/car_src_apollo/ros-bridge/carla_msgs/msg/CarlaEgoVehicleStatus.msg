#
# Copyright (c) 2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

std_msgs/Header header

geometry_msgs/Pose pose
float32 velocity
float32 acceleration
float32 curvature

# the current control values, as reported by Carla
CarlaEgoVehicleControl control
