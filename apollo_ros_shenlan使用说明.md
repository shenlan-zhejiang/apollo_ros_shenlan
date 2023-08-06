# apollo_ros_shenlan使用说明

[toc]

## 1.apollo_shenlan

### 1.1.添加第三方库

1. 复制`apollo_shenlan/third_party/`和`apollo_shenlan/third_party_shenlan/`到`apollo/`目录下

2. 对照`apollo_shenlan/tools/workspace.bzl`在`apollo/tools/workspace.bzl`中添加内容

3. 在docker内编译OMPL库

   ```sh
   cd /apollo/third_party_shenlan/repo/ompl/build/
   cmake -DCMAKE_INSTALL_PREFIX=/apollo/third_party_shenlan/ompl/ ..
   make -j8
   sudo make install
   ls /apollo/third_party_shenlan/ompl/lib/
   sudo cp -r /apollo/third_party_shenlan/ompl/lib/* /opt/apollo/sysroot/lib/
   sudo ldconfig
   ```

### 1.2.shenlan模块

1. 复制`apollo_shenlan/modules/shenlan/`到`apollo/modules/`目录下，在docker内编译shenlan

   ```sh
   bash apollo.sh build_opt shenlan
   ```

2. shenlan_mapping模块

   + 用于单独建图，需开启transform、lidar、gnss/imu、localizaiton模块
   + 读入话题：
     + 位姿：/apollo/localization/pose
     + 点云：/apollo/sensor/velodyne128/compensator/PointCloud2
   + 写出话题：
     + 栅格状态：/apollo/shenlan/mapping/occupancy
     + 融合姿点云(可视化)：/apollo/shenlan/mapping/pointcloud
     + 占据栅格(可视化)：/apollo/shenlan/mapping/gird_map

   ```sh
   cyber_launch start modules/shenlan/mapping/launch/mapping.launch
   ```

3. shenlan_minco模块

   + 用于单独规划，需开启transform、gnss/imu、localizaiton、control、canbus模块
   + 读入话题：
     + 位姿：/apollo/localization/pose
     + 栅格状态：/apollo/shenlan/mapping/occupancy
   + 写出话题：
     + 规划数据：/apollo/planning
     + kino路径(可视化)：/apollo/shenlan/minco/kino_traj
     + minco轨迹(可视化)：/apollo/shenlan/minco/minco_traj

   ```sh
   cyber_launch start modules/shenlan/minco/launch/minco.launch
   ```

4. shenlan_minco_mapping模块

   + 用于同时建图规划，需开启transform、lidar、gnss/imu、localizaiton、control、canbus模块
   + 读入话题：
     + 位姿：/apollo/localization/pose
     + 点云：/apollo/sensor/velodyne128/compensator/PointCloud2
   + 写出话题：
     + 规划数据：/apollo/planning
     + 融合姿点云(可视化)：/apollo/shenlan/mapping/pointcloud
     + 占据栅格(可视化)：/apollo/shenlan/mapping/gird_map
     + kino路径(可视化)：/apollo/shenlan/minco/kino_traj
     + minco轨迹(可视化)：/apollo/shenlan/minco/minco_traj

   ```sh
   cyber_launch start modules/shenlan/minco_mapping/launch/minco_mapping.launch
   ```

### 1.3.bridge模块

1. 替换`apollo_shenlan/modules/bridge/`到`apollo/modules/bridge/`，在docker内编译bridge

   ```sh
   bash apollo.sh build_opt bridge
   ```

2. bridge_sender模块

   + 用于apollo向ros发送数据，需配合apollo2ros_XXX使用
   + 配置文件对应bridge_sender.launch、bridge_sender.dag、udp_bridge_sender_89XX_XXX.pb.txt
   + 一个dag可以同时发送多个数据
   + 新话题需要在udp_bridge_sender_component.h和udp_bridge_sender_component.cc中注册

   ```sh
   cyber_launch start modules/bridge/launch/bridge_sender.launch
   ```

3. bridge_receiver模块

   + 用于apollo接收ros的数据，需配合ros2apollo_XXX使用
   + 配置文件对应bridge_receiver.launch、bridge_receiver_89XX_XXX.dag、udp_bridge_receiver_89XX_XXX.pb.txt
   + 一个dag只能接收一个数据，使用bridge_receiver.launch启动多个dag可以同时接受多个数据
   + 新话题需要在udp_bridge_receiver_component.h和udp_bridge_receiver_component.cc中注册

   ```sh
   cyber_launch start modules/bridge/launch/bridge_receiver.launch
   ```

## 2.apollo_ros_bridge

### 2.1.安装protobuf

1. 复制apollo_ros_shenlan/apollo_ros_bridge/文件夹到主目录下

2. 编译protobuf

   ```sh
   cd apollo_ros_bridge/protobuf-master
   sudo apt-get install autoconf automake libtool curl make g++ unzip
   ./autogen.sh 
   ./configure 
   make -j8
   sudo make install 
   sudo ldconfigs
   ```

### 2.2.apollo2ros_XXX

+ tfstatic.sh用于发布lidar到agent_0的静态坐标关系
+ apollo2ros_tf为agent_0到map的动态坐标关系，需对齐src/tf.cpp中的原点坐标
+ apollo2ros_lidar为点云数据，坐标系为lidar，需配合apollo2ros_tf和tfstatic.sh使用
+ apollo2ros_pointcloud为融合姿点云，坐标系为map，需对齐src/pointcloud.cpp中的原点坐标
+ apollo2ros_gridmap为占据栅格，坐标系为map，需对齐src/gridmap.cpp中的原点坐标
+ apollo2ros_kino为kino路径，坐标系为map，需配合src/kino.cpp中的原点坐标
+ apollo2ros_kino为minco路径，坐标系为map，需配合src/minco.cpp中的原点坐标

```sh
cd apollo_ros_bridge/apollo2ros_XXX
catkin_make
roscore
source devel/setup.bash
rosrun XXX XXX
# 或source XXX.sh
```

### 2.3.ros2apollo_XXX

```bash
cd apollo_ros_bridge/ros2apollo_XXX
catkin_make
roscore
source devel/setup.bash
rosrun XXX XXX
# 或source XXX.sh
```

## 3.planner_src

```bash
# 复制apollo_ros_shenlan/planner_planner文件夹到主目录下

# 安装osqp
cd apollo_ros_shenlan/planner_planner/osqp/build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j8
sudo make install

#安装osqp-eigen
cd apollo_ros_shenlan/planner_planner/osqp-egien/build
cmake ..
make -j8
sudo make install

# 安装ROS依赖包
# ubuntu20对应ros-noetic
sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server ros-noetic-astuff-sensor-msgs ros-noetic-ompl*
# ubuntu18对应ros-melodic
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server ros-melodic-astuff-sensor-msgs ros-melodic-ompl*

# 建立ompl文件连接
sudo ln -s /opt/ros/noetic/include/ompl-1.5/ompl /opt/ros/noetic/include/ompl

# 安装libompl
sudo apt-get install libompl-dev

# 安装ros bridge的依赖
cd planner_planner/src/ros-bridge
bash install_dependencies.sh

# 编译
catkin_make -DCMAKE_BUILD_TYPE=Release
# 可能报错并安装依赖
sudo apt-get install libgoogle-glog-dev

# 运行
cd planner_planner/
source devel/setup.bash
roslaunch traj_planner swarm_apollo.launch
# 或source planner.sh
```

## 4.carla_apollo_bridge_guardstrikelab

### 4.1.github

```sh
# apollo 7.0
https://github.com/guardstrikelab/apollo.git
# carla 9.14
https://github.com/guardstrikelab/carla_apollo_bridge.git
```

### 4.2.run apollo

```sh
# apollo docker参数：Apollo_7.0/docker/scripts/dev_into.sh & dev_start.sh
docker start carla_apollo_7.0_dev_shenlan
./Apollo_7.0/docker/scripts/dev_into.sh
bash apollo.sh build_opt
```

### 4.3.run carla simulator (server)

```bash
# 配置 carla server
# server docker参数：carla_apollo_bridge_guardstrikelab/scripts/carla-compose.yml
cd carla_apollo_bridge_guardstrikelab/scripts
./docker_run_carla.sh
docker container rename carla-simulator-1 carla_simulator_9.14

# 进入 carla server
docker start carla_simulator_9.14
# or
sh 9_simulator_carla.sh
```

### 4.4.run carla cyber (client)

```bash
# 配置 carla client
# client docker参数：carla_apollo_bridge_guardstrikelab/docker/run_docker.sh & Dockerfile
cd carla_apollo_bridge_guardstrikelab/docker
./build_docker.sh
./run_docker.sh
docker exec -ti carla_cyber_9.14 bash
./apollo.sh build_cyber opt

# 进入 carla client
docker start carla_cyber_9.14
docker exec -ti carla_cyber_9.14 bash
# or
sh 14_cyber_carla.sh

# 开启 carla bridge
# 地图参数：carla_apollo_bridge_guardstrikelab/src/carla_cyber_bridge/config/settings.yaml
cd /apollo/cyber/carla_bridge/
python carla_cyber_bridge/bridge.py
# or
sh bridge.sh
# ps -elf | grep carla_cyber_bridge/bridge.py

# 开启 carla spawn
# 车型参数：carla_apollo_bridge_guardstrikelab/src/carla_spawn_objects/config/objects.json
cd /apollo/cyber/carla_bridge
python carla_spawn_objects/carla_spawn_objects.py
# or
sh vehicle.sh
# ps -elf | grep carla_spawn_objects/carla_spawn_objects.py

# 开启 carla manual
cd /apollo/cyber/carla_bridge/carla_python/examples/
python manual_control.py 
# or 
sh manual.sh
```

### 4.5.teleop and map

```sh
# 开启 apollo teleop
./bazel-bin/moudules/canbus/tools/teleop 
# or 
sh teleop.sh

# 配置 apollo map
# 将base_map.bin生成为sim_map.bin
./bazel-bin/modules/map/tools/sim_map_generator \
-map_dir=/apollo/modules/map/data/carla_town05/ \
-output_dir=/apollo/modules/map/data/carla_town05
# 将base_map.bin生成为routing_map.bin
bash scripts/generate_routing_topo_graph.sh \
--map_dir /apollo/modules/map/data/carla_town05
```

## 5.注意事项

1.若cyber_launch无法通过ctrl+c关闭，则在终端中输入`kill -9 xxx` (xxx为 cyber_launch的进程数字)， 或输入 `ps -elf | grep mainboard`

2.编译proto：`protoc addressbook.proto --cpp_out=./`

3.`rosparam set use_sim_time false`

