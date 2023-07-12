/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <thread>

//ros
#include "ros/ros.h" //该头文件必须包含
#include "std_msgs/String.h" //ros标准消息里面的字符串消息
#include <nav_msgs/Odometry.h>

//include
#include "carla_msgs/CarlaEgoVehicleStatus.h"

//proto
#include "proto/addressbook.pb.h"
#include "proto/chassis.pb.h"
#include "proto/localization.pb.h"
#include "proto/EgoVehicleStatus.pb.h"
#include "proto/EgoVehicleControl.pb.h"

//apollo_bridge
#include "bridge/bridge_header_item.h"
#include "bridge/bridge_header.h"
#include "bridge/macro.h"
#include "bridge/udp_listener.h"
#include "bridge/util.h"
#include "bridge/bridge_proto_serialized_buf.h"

void copy_status(const std::shared_ptr<apollo::shenlan::mpc::EgoVehicleStatus> &status, carla_msgs::CarlaEgoVehicleStatusConstPtr msg)
{
    //auto pose =std::make_shared<apollo::localization::Pose>();
    
    apollo::common::PointENU position;
    position.set_x(msg->pose.position.x);
    position.set_y(msg->pose.position.y);
    position.set_z(0.1);

    apollo::common::Quaternion quaternion;
    quaternion.set_qw(msg->pose.orientation.w);
    quaternion.set_qx(msg->pose.orientation.x);
    quaternion.set_qy(msg->pose.orientation.y);
    quaternion.set_qz(msg->pose.orientation.z);

    apollo::localization::Pose pose;
    pose.mutable_position()->CopyFrom(position);
    pose.mutable_orientation()->CopyFrom(quaternion);
    status->mutable_pose()->CopyFrom(pose);

    status->set_velocity(msg->velocity);
    status->set_acceleration(msg->acceleration);
    status->set_curvature(msg->curvature);

}

void copy_control(apollo::shenlan::mpc::EgoVehicleControl &control,carla_msgs::CarlaEgoVehicleStatusConstPtr msg)
{
    //control.mutable_header()->set_sequence_num(msg->header().sequence_num());
    //control.mutable_header()->set_timestamp_sec(msg->header().timestamp_sec());

    control.set_throttle(msg->control.throttle);
    control.set_steer(msg->control.steer);
    control.set_brake(msg->control.brake);
    control.set_hand_brake(msg->control.hand_brake);
    control.set_reverse(msg->control.reverse);
    control.set_gear(msg->control.gear);
    control.set_manual_gear_shift(msg->control.manual_gear_shift);
}

static int __count__simon = 0;

void statusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr msg)
{
    std::cout << "statusCallback " << std::endl;
    const std::string remote_ip = "127.0.0.1";
    uint16_t remote_port = 8902;
    double timestamp_ = 2.0;

    auto pb_msg = std::make_shared<apollo::shenlan::mpc::EgoVehicleStatus>();
    pb_msg->mutable_header()->set_sequence_num(__count__simon);
    __count__simon++;
    //std::cout << "seq:" << msg->header.seq << " ++simon++ "<< __count__simon << std::endl;
    
    pb_msg->mutable_header()->set_timestamp_sec(timestamp_);
    apollo::shenlan::mpc::EgoVehicleControl control;
    copy_status(pb_msg, msg);
    copy_control(control,msg);
    pb_msg->mutable_control()->CopyFrom(control);



    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(remote_port);

    std::cout << "connecting to server... " << std::endl;

    int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

    int res =
        connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (res < 0) {
      std::cout << "connected server failed " << std::endl;
      return;
    }

    std::cout << "connected to server success. port [" << remote_port << "]" << std::endl;

    apollo::bridge::BridgeProtoSerializedBuf<apollo::shenlan::mpc::EgoVehicleStatus> proto_buf;
    proto_buf.Serialize(pb_msg, "EgoVehicleStatus");
    for (size_t j = 0; j < proto_buf.GetSerializedBufCount(); j++) {
      ssize_t nbytes = send(sock_fd, proto_buf.GetSerializedBuf(j),
                            proto_buf.GetSerializedBufSize(j), 0);
      if (nbytes != static_cast<ssize_t>(proto_buf.GetSerializedBufSize(j))) {
        std::cout << "sent msg failed ";
        break;
      }
      std::cout << "sent " << nbytes << " bytes to server with sequence num " << std::endl;
    }
    close(sock_fd);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "status"); //初始化ROS节点
  ros::NodeHandle n; //创建句柄节点
  ros::Subscriber sub_status = n.subscribe<carla_msgs::CarlaEgoVehicleStatusConstPtr>("/carla/agent_0/vehicle_status", 1000, statusCallback);

  ros::Rate loop_rate(100);
  while(ros::ok())
  {
      ros::spinOnce();
      //std::cout << "status" << std::endl;
      //loop_rate.sleep();
  }

  return 0;
}