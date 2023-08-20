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
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <sstream>

//ros
#include "ros/ros.h" //该头文件必须包含
#include "std_msgs/String.h" //ros标准消息里面的字符串消息
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

//proto
#include "proto/addressbook.pb.h"
#include "proto/localization.pb.h"

//apollo_bridge
#include "bridge/bridge_header_item.h"
#include "bridge/bridge_header.h"
#include "bridge/macro.h"
#include "bridge/udp_listener.h"
#include "bridge/util.h"

using apollo::bridge::BRIDGE_HEADER_FLAG;
using apollo::bridge::BridgeHeader;
using apollo::bridge::FRAME_SIZE;
using apollo::bridge::HEADER_FLAG_SIZE;
using apollo::bridge::hsize;
using apollo::bridge::MAXEPOLLSIZE;

using apollo::localization::LocalizationEstimate;

double x_, y_, z_;

struct para_t {
  ros::Publisher pub;
  int counter_pub;
  int pfd;
  uint16_t port;
};

void *pthread_handle_message(void *para_) {
  struct para_t *para = static_cast<struct para_t *>(para_);
  struct sockaddr_in client_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));
  int bytes = 0;
  int total_recv = 2 * FRAME_SIZE;
  char total_buf[2 * FRAME_SIZE] = {0};
  bytes =
      static_cast<int>(recvfrom(para->pfd, total_buf, total_recv,
                                0, (struct sockaddr *)&client_addr, &sock_len));
  if (bytes <= 0 || bytes > total_recv) {
    pthread_exit(nullptr);
  }

  // apollo
  char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1] = {0};
  size_t offset = 0;
  memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
  if (strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0) {
    std::cout << "header flag not match!" << std::endl;
    pthread_exit(nullptr);
  }
  offset += sizeof(BRIDGE_HEADER_FLAG) + 1;
  hsize header_size = *(reinterpret_cast<hsize *>(total_buf + offset));

  if (header_size > FRAME_SIZE) {
    std::cout << "header size is more than FRAME_SIZE!" << std::endl;
    pthread_exit(nullptr);
  }
  offset += sizeof(hsize) + 1;

  BridgeHeader header;
  size_t buf_size = header_size - offset;
  const char *cursor = total_buf + offset;
  if (!header.Diserialize(cursor, buf_size)) {
    std::cout << "header diserialize failed!" << std::endl;
    pthread_exit(nullptr);
  }

  /*
  std::cout << "proto name : " << header.GetMsgName().c_str() << std::endl;
  std::cout << "proto sequence num: " << header.GetMsgID() << std::endl;
  std::cout << "proto total frames: " << header.GetTotalFrames() << std::endl;
  std::cout << "proto frame index: " << header.GetIndex() << std::endl;
  std::cout << "proto size: " << header.GetMsgSize() << std::endl;
  std::cout << "proto frame pos: " << header.GetFramePos() << std::endl;
  */

  apollo::localization::LocalizationEstimate obj2;
  int offset_data = header_size - header.GetFramePos();
  obj2.ParseFromArray(total_buf + offset_data, header.GetMsgSize());

  /*
  if (1) {
    std::cout << "sequence num: " << obj2.header().sequence_num() << std::endl;
    std::cout << "timestamp sec: " << obj2.header().timestamp_sec() << std::endl;
    std::cout << "position x: " << obj2.pose().position().x() << std::endl;
    std::cout << "position y: " << obj2.pose().position().y() << std::endl;
    std::cout << "position z: " << obj2.pose().position().z() << std::endl;
    std::cout << "orientation x: " << obj2.pose().orientation().qx() << std::endl;
    std::cout << "orientation y: " << obj2.pose().orientation().qy() << std::endl;
    std::cout << "orientation z: " << obj2.pose().orientation().qz() << std::endl;
    std::cout << "orientation w: " << obj2.pose().orientation().qw() << std::endl;
    std::cout << "linear vx: " << obj2.pose().linear_velocity().x() << std::endl;
    std::cout << "linear vy: " << obj2.pose().linear_velocity().y() << std::endl;
    std::cout << "linear vz: " << obj2.pose().linear_velocity().z() << std::endl;
    std::cout << "angular vx: " << obj2.pose().angular_velocity().x() << std::endl;
    std::cout << "angular vy: " << obj2.pose().angular_velocity().y() << std::endl;
    std::cout << "angular vz: " << obj2.pose().angular_velocity().z() << std::endl;
  }
  */

  // tf::TransformBroadcaster odom_broadcaster;
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = ros::Time(obj2.header().timestamp_sec());
  // odom_trans.header.frame_id = "map";
  // odom_trans.child_frame_id = "agent_0";

  // odom_broadcaster.sendTransform(odom_trans);

  // rostopic echo odmetry
  nav_msgs::Odometry odom_msg_;
  odom_msg_.header.stamp = ros::Time::now(); //(obj2.header().timestamp_sec());
  // odom_msg_.header.stamp = ros::Time(obj2.header().timestamp_sec());
  odom_msg_.header.seq = obj2.header().sequence_num();
  odom_msg_.header.frame_id = "map";
  odom_msg_.child_frame_id = "agent_0";

  odom_msg_.pose.pose.position.x  = obj2.pose().position().x() - x_;
  odom_msg_.pose.pose.position.y  = obj2.pose().position().y() - y_;
  odom_msg_.pose.pose.position.z  = obj2.pose().position().z() - z_;

  odom_msg_.pose.pose.orientation.x = obj2.pose().orientation().qx();
  odom_msg_.pose.pose.orientation.y = obj2.pose().orientation().qy();
  odom_msg_.pose.pose.orientation.z = obj2.pose().orientation().qz();
  odom_msg_.pose.pose.orientation.w = obj2.pose().orientation().qw();

  odom_msg_.twist.twist.linear.x = obj2.pose().linear_velocity().x();
  odom_msg_.twist.twist.linear.y = obj2.pose().linear_velocity().y();
  odom_msg_.twist.twist.linear.z = obj2.pose().linear_velocity().z();
  
  odom_msg_.twist.twist.angular.x = obj2.pose().angular_velocity().x();
  odom_msg_.twist.twist.angular.y = obj2.pose().angular_velocity().y();
  odom_msg_.twist.twist.angular.z = obj2.pose().angular_velocity().z();

  // std::cout << "odom_msg_.header.seq : " << obj2.header().sequence_num() << "====" << odom_msg_.header.seq << std::endl;
  // odom_msg_.header.seq = obj2.header().sequence_num();

  std_msgs::String msg;
  para->pub.publish(odom_msg_);
  // std::stringstream ss;//创建一个stringstream对象
  // ss << std::fixed << std::setprecision(9) << para->counter_pub << " " << odom_msg_.header.stamp << " " << obj2.header().sequence_num() << " " << obj2.header().timestamp_sec();//字符串拼接，把拼接后的数值取出来赋值给我们的msg
  // msg.data = ss.str();//可以把string流里面的数据提取为字符串
  // para->relation_pub.publish(msg);//发布msg
  
  // std::cout << "after==point_cloud_.header.seq : " << obj2.header().sequence_num() << "====" << odom_msg_.header.seq << std::endl;
  // std::cout << "+++++++++" << para->counter_pub << "==========" << odom_msg_.header.seq << "======" << obj2.header().timestamp_sec() << std::endl;
  std::cout << "header: " << odom_msg_.header << std::endl;
  std::cout << "child_frame_id: " << odom_msg_.child_frame_id << std::endl;
  std::cout << "pose: " << odom_msg_.pose.pose << std::endl;
  // std::cout << odom_msg_.twist.twist << std::endl;

  para->counter_pub++;
  pthread_exit(nullptr);
}

bool receive(struct para_t *para) {
  struct rlimit rt;
  rt.rlim_max = rt.rlim_cur = MAXEPOLLSIZE;
  if (setrlimit(RLIMIT_NOFILE, &rt) == -1) {
    std::cout << "set resource limitation failed" << std::endl;
    return false;
  }

  int listener_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (listener_sock == -1) {
    std::cout << "create socket failed" << std::endl;
    return false;
  }
  int opt = SO_REUSEADDR;
  setsockopt(listener_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  if (fcntl(listener_sock, F_SETFL,
            fcntl(listener_sock, F_GETFD, 0) | O_NONBLOCK) == -1) {
    std::cout << "set nonblocking failed" << std::endl;
    return false;
  }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = PF_INET;
  serv_addr.sin_port = htons(para->port);
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(listener_sock, (struct sockaddr *)&serv_addr,
           sizeof(struct sockaddr)) == -1) {
    close(listener_sock);
    std::cout << "bind socket failed" << std::endl;
    return false;
  }
  int kdpfd = epoll_create(MAXEPOLLSIZE);
  struct epoll_event ev;
  ev.events = EPOLLIN | EPOLLET;
  ev.data.fd = listener_sock;
  if (epoll_ctl(kdpfd, EPOLL_CTL_ADD, listener_sock, &ev) < 0) {
    std::cout << "set control interface for an epoll descriptor failed" << std::endl;
    close(listener_sock);
    return false;
  }

  std::cout << "Ready!" << std::endl;

  int nfds = -1;
  bool res = true;
  struct epoll_event events[MAXEPOLLSIZE];
  while (true) {
    nfds = epoll_wait(kdpfd, events, 10000, -1);
    if (nfds == -1) {
      std::cout << "some error occurs while waiting for I/O event" << std::endl;
      res = false;
      break;
    }

    for (int i = 0; i < nfds; ++i) {
      if (events[i].data.fd == listener_sock) {
        pthread_t thread;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        para->pfd = events[i].data.fd;
        if (pthread_create(&thread, &attr, &pthread_handle_message,
                           reinterpret_cast<void *>(para))) {
          std::cout << "message handler creation failed" << std::endl;
          res = false;
          break;
        }
      }
    }
    if (!res) { 
      break;
    }
  }
  close(listener_sock);
  return res;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "odometry"); //节点名为study，随便取
  ros::NodeHandle n;

  n.param("x", x_, 0.0);
  n.param("y", y_, 0.0);
  n.param("z", z_, 0.0);

  struct para_t para;
  para.pub = n.advertise<nav_msgs::Odometry>("/apollo/agent_0/odometry", 1000);
  //para.relation_pub = n.advertise<std_msgs::String>("/apollo/agent_0/relation_odom", 1000);
  para.port = 8901;
  para.counter_pub = 0;
  para.pfd = -1;

  receive(&para);

  return 0;
}
