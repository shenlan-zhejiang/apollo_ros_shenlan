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
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

//proto
#include "proto/addressbook.pb.h"
#include "proto/transform.pb.h"
#include "proto/static_transform_conf.pb.h"

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

using apollo::transform::TransformStampeds;

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

  apollo::transform::TransformStampeds obj2;
  int offset_data = header_size - header.GetFramePos();
  obj2.ParseFromArray(total_buf + offset_data, header.GetMsgSize());

/*
if (1) {
  // std::cout << "sequence num: " << obj2.header().sequence_num() << std::endl;
  // std::cout << "timestamp sec: " << obj2.header().timestamp_sec() << std::endl;
  // std::cout << "size " << obj2.transforms_size() << std::endl;
  for (auto it : obj2.transforms()) {
    std::cout << "timestamp sec: " << it.header().timestamp_sec() << std::endl;
    std::cout << "child_frame_id: " << it.child_frame_id() << std::endl;

  }
}
*/

  // rostopic echo tf

  tf2_msgs::TFMessage tf_;

  for (auto it : obj2.transforms()) {
    geometry_msgs::TransformStamped tfs_;
    tfs_.header.stamp = ros::Time::now(); //(it.header().timestamp_sec());
    // tfs_.header.stamp = ros::Time(obj2.header().timestamp_sec());
    tfs_.header.seq = obj2.header().sequence_num();
    tfs_.header.frame_id = "map"; 
    tfs_.child_frame_id = "agent_0";
    tfs_.transform.translation.x = it.transform().translation().x() - x_;
    tfs_.transform.translation.y = it.transform().translation().y() - y_;
    tfs_.transform.translation.z = it.transform().translation().z() - z_;
    tfs_.transform.rotation.x = it.transform().rotation().qx();
    tfs_.transform.rotation.y = it.transform().rotation().qy();
    tfs_.transform.rotation.z = it.transform().rotation().qz();
    tfs_.transform.rotation.w = it.transform().rotation().qw();
    tf_.transforms.emplace_back(tfs_);
  }
    

  // for (auto it : obj2.transforms()) {
  //   geometry_msgs::TransformStamped tfs_;
  //   tfs_.header.stamp = ros::Time(it.header().timestamp_sec());
  //   tfs_.header.frame_id = "map"; 
  //   tfs_.child_frame_id = "lidar";
  //   tfs_.transform.translation.x = 0;
  //   tfs_.transform.translation.y = 0;
  //   tfs_.transform.translation.z = 2;
  //   tfs_.transform.rotation.x = 0;
  //   tfs_.transform.rotation.y = 0;
  //   tfs_.transform.rotation.z = 0.7071;
  //   tfs_.transform.rotation.w = 0.7071;
  //   tf_.transforms.emplace_back(tfs_);
  // }

  // std::cout << "tfs_.header.seq : " << obj2.header().sequence_num() << "====" << tfs_.header.seq << std::endl;
  // tfs_.header.seq = obj2.header().sequence_num();
  para->pub.publish(tf_);
  // std::cout << "after==tfs_.header.seq : " << obj2.header().sequence_num() << "====" << tfs_.header.seq << std::endl;
  std::cout << tf_ << std::endl;

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
  ros::init(argc, argv, "tf"); //节点名为study，随便取
  ros::NodeHandle n;

  n.param("x", x_, 0.0);
  n.param("y", y_, 0.0);
  n.param("z", z_, 0.0);

  struct para_t para;
  para.pub = n.advertise<tf2_msgs::TFMessage>("/tf", 1000);
  //para.relation_pub = n.advertise<std_msgs::String>("/apollo/agent_0/relation_odom", 1000);
  para.port = 8903;
  para.counter_pub = 0;
  para.pfd = -1;

  receive(&para);

  return 0;
}
