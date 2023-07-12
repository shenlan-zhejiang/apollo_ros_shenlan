
#ifndef SRC_MPC_INCLUDE_UTILS_CARLA_ADAPTER_H_
#define SRC_MPC_INCLUDE_UTILS_CARLA_ADAPTER_H_

#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>

int GetPhysicParamFromCarlaRosBridge(const ros::NodeHandle &nh, const std::string &vehicle_name){
  auto vehicle_info = ros::topic::waitForMessage<carla_msgs::CarlaEgoVehicleInfo>("/carla/"+vehicle_name+"/vehicle_info",
                                                                                  ros::Duration(5));

  unsigned int front_idx = 0;
  unsigned int rear_idx = 2;
  if(vehicle_info->wheels.size() == 2) {
    rear_idx = 1;
  }

  double max_steer_angle = vehicle_info->wheels.at(0).max_steer_angle;

  double wheel_base = vehicle_info->wheels.at(0).position.x - vehicle_info->wheels.at(2).position.x;
  double length_front = std::abs(vehicle_info->wheels.at(0).position.x - vehicle_info->center_of_mass.x);
  double length_rear = std::abs(vehicle_info->wheels.at(2).position.x - vehicle_info->center_of_mass.x);

  if(wheel_base - (length_front + length_rear) > DBL_EPSILON){
    // TODO: Use fatal error
    std::cout << "Error: Wheel base != length_front + length_rear" << std::endl;
  }

  double mass_front = length_front / wheel_base * vehicle_info->mass;
  double mass_rear = length_rear / wheel_base * vehicle_info->mass;
  double inertial_z = length_front * length_front * mass_front + length_rear * length_rear * mass_rear;

  std::cout << "wheel_base: " << wheel_base << std::endl;
  std::cout << "mass_f / 2.0: " << (mass_front / 2.0) << std::endl;;
  std::cout << "mass_r / 2.0: " << (mass_rear / 2.0)<< std::endl;
  std::cout << "length_rear: " << fabs(vehicle_info->wheels.at(2).position.x)<< std::endl;

  return 0;
}

#endif //SRC_MPC_INCLUDE_UTILS_CARLA_ADAPTER_H_