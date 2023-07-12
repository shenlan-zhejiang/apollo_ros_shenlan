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
#include <sensor_msgs/PointCloud2.h>

//proto
#include "proto/addressbook.pb.h"
#include "proto/pointcloud.pb.h"

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

using apollo::drivers::PointCloud;

#define CAPACITY  128

struct para_t {
  ros::Publisher pub;
  int pfd;
  uint16_t port;
  char *buf[CAPACITY];
  int seq[CAPACITY]; //int ?????????????//
  int count[CAPACITY];
  int total[CAPACITY];
  int size[CAPACITY];
  int cap;
};

void *handle_message(void *para_) {
  struct para_t *para = static_cast<struct para_t *>(para_);
  struct sockaddr_in client_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));
  int bytes = 0;
  int total_recv = 2 * FRAME_SIZE;
  char total_buf[2 * FRAME_SIZE] = {0};
  bytes =
      static_cast<int>(recvfrom(para->pfd, total_buf, total_recv,
                                0, (struct sockaddr *)&client_addr, &sock_len));
  
  double end_time = ros::Time::now().toSec();
  std::cout << "length:"  <<  bytes << " time:" << end_time << "error:" << errno << std::endl;

  if (bytes <= 0 || bytes > total_recv) {
    return nullptr; 
  }

  char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1] = {0};
  size_t offset = 0;
  memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
  if (strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0) {
    std::cout << "header flag not match!" << std::endl;
    return nullptr; 
  }
  offset += sizeof(BRIDGE_HEADER_FLAG) + 1;
  hsize header_size = *(reinterpret_cast<hsize *>(total_buf + offset));

  if (header_size > FRAME_SIZE) {
    std::cout << "header size is more than FRAME_SIZE!" << std::endl;
    return nullptr; 
  }
  offset += sizeof(hsize) + 1;

  BridgeHeader header;
  size_t buf_size = header_size - offset;
  const char *cursor = total_buf + offset;
  if (!header.Diserialize(cursor, buf_size)) {
    std::cout << "header diserialize failed!" << std::endl;
    return nullptr; 
  }

  // std::cout << "proto name : " << header.GetMsgName().c_str() << std::endl;
  // std::cout << "sequence num: " << obj2.header().sequence_num() << std::endl;
  // std::cout << "timestamp sec: " << obj2.header().timestamp_sec() << std::endl;

  std::cout << "proto name : " << header.GetMsgName().c_str() << std::endl;
  std::cout << "proto sequence num: " << header.GetMsgID() << std::endl;
  std::cout << "proto total frames: " << header.GetTotalFrames() << std::endl;
  std::cout << "proto frame index: " << header.GetIndex() << std::endl;
  std::cout << "proto size: " << header.GetMsgSize() << std::endl;
  std::cout << "proto frame pos: " << header.GetFramePos() << std::endl;

  int idx;
  for (idx = 0; idx < CAPACITY; idx++) {
    if (para->seq[idx] == header.GetMsgID()) break; 
  }
  if (idx == CAPACITY && para->cap == CAPACITY) {
    std::cout << "buf is not enough!" << std::endl;
    for (int _idx = 0; _idx < CAPACITY; _idx++) {
      std::cout << "idx:" << _idx << " seq: " << para->seq[_idx] << " count: " << para->count[_idx] << " total: "<< para->total[_idx] << std::endl;
    }
    exit(0);  // this line need be removed
    return nullptr;
  }
  if (idx == CAPACITY) {
    for (idx = 0; idx < CAPACITY; idx++) {
      if (para->seq[idx] == -1) {
        para->seq[idx] = header.GetMsgID();
        para->total[idx] = header.GetTotalFrames();
        para->count[idx] = 0;
        para->cap += 1;
        para->buf[idx] = new char[header.GetMsgSize()]; //?
        para->size[idx] = header.GetMsgSize();
        break;
      }
    }
  }
  
  memcpy(para->buf[idx] + header.GetFramePos(), total_buf + header_size, header.GetFrameSize());
  para->count[idx] += 1;
  std::cout << "I:" << idx << " count:" << para->count[idx] << "total: " << para->total[idx] <<" fsize:" << FRAME_SIZE << " pos:" << header.GetFramePos() << "fs:" << header.GetFrameSize() << std::endl;

  return nullptr;
}

bool parse_data(struct para_t *para)
{
  int idx;
  for (idx = 0; idx < CAPACITY; idx++) {
    if (para->count[idx] == para->total[idx] && para->seq[idx] != -1) break;
  }
  if (idx >= CAPACITY) return false;
  std::cout << "I:" << idx << " count:" << para->count[idx] << "total: " << para->total[idx] << std::endl;

  // apollo::localization::LocalizationEstimate obj2;
  apollo::drivers::PointCloud obj2;

  obj2.ParseFromArray(para->buf[idx], para->size[idx]);
  delete para->buf[idx];
  para->cap -= 1;
  para->seq[idx] = -1;

  if (1) {
    std::cout << "sequence num: " << obj2.header().sequence_num() << std::endl;
    std::cout << "timestamp sec: " << obj2.header().timestamp_sec() << std::endl;
    std::cout << "width: " << obj2.width() << std::endl;
    std::cout << "height: " << obj2.height() << std::endl;
  }

  // rostopic echo lidar
  sensor_msgs::PointCloud2 point_cloud2_;
  point_cloud2_.header.stamp = ros::Time(obj2.header().timestamp_sec()); //apollo time
  point_cloud2_.header.frame_id = "agent_0/lidar";
  point_cloud2_.width = obj2.width();
  point_cloud2_.height = obj2.height();

  para->pub.publish(point_cloud2_);

  //pthread_exit(nullptr);
  return true;
}

bool receive(struct para_t *para) {
  errno = 0;
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

    /*
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
        ev.events= EPOLLIN | EPOLLET;
        if (epoll_ctl(kdpfd, EPOLL_CTL_MOD, listener_sock, &ev) < 0) {
          std::cout << "set2222 control interface for an epoll descriptor failed" << std::endl;
          close(listener_sock);
          return false;
        }
      }
    }
    //*/

    for (int i = 0; i < nfds; ++i) {
      if (events[i].data.fd == listener_sock) {
        para->pfd = events[i].data.fd;
        handle_message(reinterpret_cast<void *>(para));
        ev.events= EPOLLIN | EPOLLET;
        if (epoll_ctl(kdpfd, EPOLL_CTL_MOD, listener_sock, &ev) < 0) {
          std::cout << "set2222 control interface for an epoll descriptor failed" << std::endl;
          close(listener_sock);
          return false;
        }
        parse_data(para);
      }
    }
    if (!res) {
      break;
    }
  }
  std::cout << "while finished" << std::endl;
  close(listener_sock);
  return res;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test1"); //节点名为study，随便取
  ros::NodeHandle n;
  struct para_t para;
  memset(para.buf, 0, sizeof(char *) * CAPACITY);
  for (int idx = 0; idx < CAPACITY; idx++) {
    para.seq[idx] = -1;
  }
  para.cap = 0;
  para.pub = n.advertise<sensor_msgs::PointCloud2>("/carla/agent_0/lidar", 1000);
  para.port = 8903;
  para.pfd = -1;
  receive(&para);
  return 0;
}
