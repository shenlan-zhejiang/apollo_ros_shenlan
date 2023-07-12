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
#include "common/traj_anal.hpp"

//proto
#include "proto/Trajectory.pb.h"

//apollo_bridge
#include "bridge/bridge_header_item.h"
#include "bridge/bridge_header.h"
#include "bridge/macro.h"
#include "bridge/udp_listener.h"
#include "bridge/util.h"
#include "bridge/bridge_proto_serialized_buf.h"

static void copy_nav(apollo::shenlan::mpc::Nav_path &path, mpc::TrajectoryConstPtr msg)
{
    auto _path = msg->nav_path;
    path.mutable_header()->set_sequence_num(_path.header.seq);

    for (auto & _pose : _path.poses) {    
        auto pose = path.add_pose();

        apollo::common::PointENU position;
        position.set_x(_pose.pose.position.x);
        position.set_y(_pose.pose.position.y);
        position.set_z(_pose.pose.position.z);
        pose->mutable_position()->CopyFrom(position);

        apollo::common::Quaternion quaternion;
        quaternion.set_qw(_pose.pose.orientation.w);
        quaternion.set_qx(_pose.pose.orientation.x);
        quaternion.set_qy(_pose.pose.orientation.y);
        quaternion.set_qz(_pose.pose.orientation.z);
        pose->mutable_orientation()->CopyFrom(quaternion);
    }
}

static void copy_minco(apollo::shenlan::mpc::MincoTraj &minco, mpc::TrajectoryConstPtr msg)
{
    auto _minco = msg->minco_path;
    //minco.mutable_header()->set_sequence_num(msg->header.seq);

    for (auto & _single : _minco.trajs) {    
        auto single = minco.add_trajs();
        single->set_start_time(_single.start_time.toSec());

        apollo::common::Point3D point;
        point.set_x(_single.head_x.x);
        point.set_y(_single.head_x.y);
        point.set_z(_single.head_x.z);
        single->mutable_head_x()->CopyFrom(point);

        point.set_x(_single.head_y.x);
        point.set_y(_single.head_y.y);
        point.set_z(_single.head_y.z);
        single->mutable_head_y()->CopyFrom(point);

        point.set_x(_single.tail_x.x);
        point.set_y(_single.tail_x.y);
        point.set_z(_single.tail_x.z);
        single->mutable_tail_x()->CopyFrom(point);

        point.set_x(_single.tail_y.x);
        point.set_y(_single.tail_y.y);
        point.set_z(_single.tail_y.z);
        single->mutable_tail_y()->CopyFrom(point);

        for (auto & _point : _single.pos_pts) {
            auto point = single->add_pos_pts();
            point->set_x(_point.x);
            point->set_y(_point.y);
            point->set_z(_point.z);
        }
        for (auto & _t : _single.t_pts) {
            single->add_t_pts(_t);
        }
        single->set_reverse(_single.reverse);
    }
}

static int __count__simon = 0;

void trajCallback(const mpc::TrajectoryConstPtr msg)
{
    std::cout << "trajCallback " << std::endl;
    const std::string remote_ip = "127.0.0.1";
    uint16_t remote_port = 8903;
    double timestamp_ = 2.0;

    auto pb_msg = std::make_shared<apollo::shenlan::mpc::Trajectory>();
    pb_msg->mutable_header()->set_sequence_num(__count__simon);
    __count__simon++;
    //std::cout << "seq:" << msg->nav_path.header.seq << std::endl;
    
    pb_msg->mutable_header()->set_timestamp_sec(timestamp_);
    apollo::shenlan::mpc::Nav_path nav;
    apollo::shenlan::mpc::MincoTraj minco;
    copy_nav(nav, msg);
    copy_minco(minco, msg);
    pb_msg->mutable_nav_path()->CopyFrom(nav);
    pb_msg->mutable_minco_path()->CopyFrom(minco);



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

    apollo::bridge::BridgeProtoSerializedBuf<apollo::shenlan::mpc::Trajectory> proto_buf;
    proto_buf.Serialize(pb_msg, "Trajectory");
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
  ros::init(argc, argv, "trajectory"); //初始化ROS节点
  ros::NodeHandle n; //创建句柄节点
  ros::Subscriber sub_traj = n.subscribe<mpc::TrajectoryConstPtr>("/carla/agent_0/trajectory", 1000, trajCallback);

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
      ros::spinOnce();
      //std::cout << "traj" << std::endl;
      //loop_rate.sleep();
  }

  return 0;
}