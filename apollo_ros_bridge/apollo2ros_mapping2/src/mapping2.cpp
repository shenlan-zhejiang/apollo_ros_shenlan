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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>

//proto
#include "proto/addressbook.pb.h"
#include "proto/pointcloud.pb.h"
#include "proto/localization.pb.h"

//apollo_bridge
#include "bridge/bridge_header_item.h"
#include "bridge/bridge_header.h"
#include "bridge/macro.h"
#include "bridge/udp_listener.h"
#include "bridge/util.h"

#include "cyber/clock.h"
// #include "cyber/common/log.h"
// #include "cyber/scheduler/scheduler_factory.h"

using apollo::bridge::BRIDGE_HEADER_FLAG;
using apollo::bridge::BridgeHeader;
using apollo::bridge::FRAME_SIZE;
using apollo::bridge::HEADER_FLAG_SIZE;
using apollo::bridge::hsize;
using apollo::bridge::MAXEPOLLSIZE;

using apollo::drivers::PointCloud;
using apollo::localization::LocalizationEstimate;

using apollo::cyber::Clock;

#define CAPACITY  4

struct para_t {
  ros::Publisher pub;
  ros::Publisher relation_pub;
  int counter_pub;
  int pfd;
  int index;
  pthread_mutex_t mutex;
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
  
  // std::cout << "length:"  <<  bytes << " time:" << end_time << "error:" << errno << std::endl;

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
  // std::cout << "proto sequence num: " << header.GetMsgID() << std::endl;
  // std::cout << "proto total frames: " << header.GetTotalFrames() << std::endl;
  // std::cout << "proto frame index: " << header.GetIndex() << std::endl;
  // std::cout << "proto size: " << header.GetMsgSize() << std::endl;
  // std::cout << "proto frame pos: " << header.GetFramePos() << std::endl;

  int idx;
  for (idx = 0; idx < CAPACITY; idx++) {
    if (para->seq[idx] == header.GetMsgID()) break; 
  }
  if (idx == CAPACITY && para->cap == CAPACITY) {
    std::cout << "buf is not enough!" << std::endl;
    int pos = 0;
    for (int _idx = 1; _idx < CAPACITY; _idx++) {
      if (para->seq[pos] > para->seq[_idx]) { pos = _idx; }
      //std::cout << "idx:" << _idx << " seq: " << para->seq[_idx] << " count: " << para->count[_idx] << " total: "<< para->total[_idx] << std::endl;
    }
    //delete para->buf[pos];//
    para->cap -= 1;
    para->seq[pos] = -1;
    //exit(0);  // this line need be removed
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
  // std::cout << "C:" << para->cap << "I:" << idx << " count:" << para->count[idx] << "total: " << para->total[idx] <<" fsize:" << FRAME_SIZE << " pos:" << header.GetFramePos() << "fs:" << header.GetFrameSize() << std::endl;

  return nullptr;
}

void *parse_data(void *para_)
{
  struct para_t *para = static_cast<struct para_t *>(para_);
  int idx = para->index;
  pthread_mutex_unlock(&para->mutex);
  apollo::drivers::PointCloud obj2;

  // std::cout << "C:" << para->cap << "I:" << idx << " count:" << para->count[idx] << "total: " << para->total[idx] << std::endl;
  
  obj2.ParseFromArray(para->buf[idx], para->size[idx]);
  //delete para->buf[idx];
  para->cap -= 1;
  para->seq[idx] = -1;

  /*
  if (1) {
    std::cout << "sequence num: " << obj2.header().sequence_num() << std::endl;
    std::cout << "timestamp sec: " << obj2.header().timestamp_sec() << std::endl;
    std::cout << "width: " << obj2.width() << std::endl;
    std::cout << "height: " << obj2.height() << std::endl;
    std::cout << "ts" << ros::Time::now().toSec() << std::endl;
    std::cout << "t" << ros::Time::now() << std::endl;
    std::cout << "wt" << ros::WallTime::now() << std::endl;
  }
  */

  // rostopic echo mapping2
  sensor_msgs::PointCloud point_cloud_;
  std::string str_timestamps;
  point_cloud_.header.seq = obj2.header().sequence_num();
  // point_cloud_.header.stamp = ros::Time(obj2.header().timestamp_sec());
  point_cloud_.header.stamp = ros::Time::now(); // Clock::NowInSeconds(); //ros::Time::now(); //(obj2.header().timestamp_sec()); //apollo time Clock::NowInSeconds()
  point_cloud_.header.frame_id = "lidar";
  point_cloud_.channels.resize(1);
  point_cloud_.channels[0].name = "intensities";

  // std::cout << Clock::NowInSeconds() << std::endl;

  for (auto iter : obj2.point()) {
    geometry_msgs::Point32 pt;
    pt.x = iter.x();
    pt.y = iter.y();
    pt.z = iter.z();
    point_cloud_.points.emplace_back(pt);
    point_cloud_.channels[0].values.emplace_back(iter.intensity());
  }

  sensor_msgs::PointCloud2 point_cloud2_;
  // point_cloud2_.header.stamp = ros::Time(obj2.header().timestamp_sec()); //apollo time
  // point_cloud2_.header.frame_id = "agent_0/mapping2";
  convertPointCloudToPointCloud2(point_cloud_, point_cloud2_);

  // std::cout << "point_cloud_.header.seq : " << obj2.header().sequence_num() << "====" << point_cloud_.header.seq << std::endl;
  // point_cloud_.header.seq = obj2.header().sequence_num();

  std_msgs::String msg;
  para->pub.publish(point_cloud2_);
  std::stringstream ss;//创建一个stringstream对象
  ss << std::fixed << std::setprecision(9) << para->counter_pub << " " << point_cloud2_.header.stamp << " " << obj2.header().sequence_num() << " " << obj2.header().timestamp_sec();//字符串拼接，把拼接后的数值取出来赋值给我们的msg
  msg.data = ss.str();//可以把string流里面的数据提取为字符串
  para->relation_pub.publish(msg);//发布msg

  // std::cout << "after==point_cloud_.header.seq : " << obj2.header().sequence_num() << "====" << point_cloud2_.header.seq << std::endl;
  // std::cout << "+++++++++" << para->counter_pub << "==========" << point_cloud_.header.seq << std::endl;
  std::cout << "header: " << point_cloud2_.header << std::endl;
  std::cout << "width: " << point_cloud2_.width << std::endl;
  std::cout << "height: " << point_cloud2_.height << std::endl;

  para->counter_pub++;

  pthread_exit(nullptr);
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

        for (int idx = 0; idx < CAPACITY; idx++) {
          if (para->count[idx] == para->total[idx] && para->seq[idx] != -1) {
            pthread_t thread;
            pthread_attr_t attr;
            pthread_attr_init(&attr);
            pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
            pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
            // std::cout << "para->mutex: " << idx << std::endl;
            if (pthread_mutex_trylock(&para->mutex)) {continue;}
            // std::cout << "para->mutex" << std::endl;
            para->index = idx;   //
            if (pthread_create(&thread, &attr, &parse_data,
                              reinterpret_cast<void *>(para))) {
              std::cout << "message handler creation failed" << std::endl;
              res = false;
              break;
            }
          }
        }
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
  ros::init(argc, argv, "mapping2"); //节点名为study，随便取
  ros::NodeHandle n;
  struct para_t para;
  memset(para.buf, 0, sizeof(char *) * CAPACITY);
  for (int idx = 0; idx < CAPACITY; idx++) {
    para.seq[idx] = -1;
  }
  para.cap = 0;
  para.pub = n.advertise<sensor_msgs::PointCloud2>("/apollo/agent_0/mapping2", 1000);
  para.relation_pub = n.advertise<std_msgs::String>("/apollo/agent_0/relation_mapping2", 1000);
  para.counter_pub = 0;
  para.port = 8906;
  para.pfd = -1;
  para.index = -1;
  pthread_mutex_init(&para.mutex, NULL);

  receive(&para);
  return 0;
}
