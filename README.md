# apollo_ros_shenlan使用说明

[toc]

## 1.apollo_shenlan

+ apollo_shenlan是基于APOLLO的轻量级自动驾驶系统，实现的功能有实时栅格地图构建、前端路径规划、后端轨迹优化等，主要包括shenlan模块、bridge模块

  + shenlan模块用于建图规划，包括mapping组件、minco组件、minco_mapping组件

  + bridge模块用于跨平台通信，包括bridge_sender组件、bridge_receiver组件

### 1.1.添加第三方库

1. shenlan模块依赖于ompl、osqp2、oqspeigen第三方库，ompl需要在docker内编译安装，osqp2和oqspeigen直接导入即可使用

1. 复制`apollo_shenlan/third_party`、`apollo_shenlan/third_party_shenlan`到`apollo`目录下

2. 对照`apollo_shenlan/tools/workspace.bzl`在`apollo/tools/workspace.bzl`中添加内容

3. 在docker内编译ompl库

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

1. 复制`apollo_shenlan/modules/shenlan`、`apollo_shenlan/modules/canbus`、`apollo_shenlan/modules/dreamview`到`apollo/modules`目录下，在docker内编译shenlan

   ```sh
   ./apollo.sh build_opt shenlan
   ```

2. conf配置文件

   + `modules/shenlan/conf`文件夹下提供了不同场景的配置参数，包括车型、建图、状态机、路径规划、轨迹优化的参数
   + 场景包括：实车的huzhou，SVL仿真的borregasave、sanfrancisco、shalun，CARLA仿真的town01、town04、town10
   + 使用方法：将shenlan_conf_xxx.pb.txt内的全部内容复制到shenlan_conf.pb.txt内，并修改mapping.dag和minco_mapping.dag中的点云话题

3. mapping组件

   + 用于单独建图，需开启transform、lidar、gnss/imu、localizaiton模块
   + 读入话题(在mapping.dag中修改)：
     + 位姿：/apollo/localization/pose
     + 点云：/apollo/sensor/lidar/compensator/PointCloud2
   + 输出话题(在mapping_component.cc中修改)：
     + 融合位姿点云(可视化)：/apollo/shenlan/mapping/pointcloud
     + 占据栅格(可视化)：/apollo/shenlan/mapping/grid_map
     + 栅格状态：/apollo/shenlan/mapping/occupancy

   ```sh
   cyber_launch start modules/shenlan/mapping/launch/mapping.launch
   
   # 或点击dreamview中ApolloShenlan调试模式下的Mapping按钮
   ```

4. minco组件

   + 用于单独规划，需开启transform、gnss/imu、localizaiton、control、canbus模块
   + 读入话题(在minco.dag中修改)：
     + 位姿：/apollo/localization/pose
     + 栅格状态：/apollo/shenlan/mapping/occupancy
   + 输出话题(在minco_component.cc中修改)：
     + kino轨迹(可视化)：/apollo/shenlan/minco/kino_traj
     + minco轨迹(可视化)：/apollo/shenlan/minco/minco_traj
     + 规划数据：/apollo/planning

   ```sh
   cyber_launch start modules/shenlan/minco/launch/minco.launch
   
   # 或点击dreamview中ApolloShenlan调试模式下的Minco按钮
   ```

5. minco_mapping组件

   + 用于同时建图规划，需开启transform、lidar、gnss/imu、localizaiton、control、canbus模块
   + 读入话题(在minco_mapping.dag中修改)：
     + 位姿：/apollo/localization/pose
     + 点云：/apollo/sensor/lidar/compensator/PointCloud2
   + 输出话题(在minco_mapping_component.cc中修改)：
     + 融合位姿点云(可视化)：/apollo/shenlan/mapping/pointcloud
     + 占据栅格(可视化)：/apollo/shenlan/mapping/grid_map
     + kino轨迹(可视化)：/apollo/shenlan/minco/kino_traj
     + minco轨迹(可视化)：/apollo/shenlan/minco/minco_traj
     + 原始规划数据：/apollo/shenlan/minco/mpc_trajectory
     + 规划数据：/apollo/planning

   ```sh
   cyber_launch start modules/shenlan/minco_mapping/launch/minco_mapping.launch
   
   # 或点击dreamview中ApolloShenlan调试模式下的MincoMapping按钮
   ```

### 1.3.bridge模块

1. 替换`apollo_shenlan/modules/bridge`到`apollo/modules/bridge`，在docker内编译bridge

   ```sh
   ./apollo.sh build_opt bridge
   ```

2. bridge_sender组件

   + 用于APOLLO向ROS发送数据，需配合ROS的apollo2ros模块使用
   + 配置文件对应launch/bridge_sender.launch、dag/bridge_sender.dag、conf/udp_bridge_sender_89xx_xxx.pb.txt
   + 一个dag可以同时发送多个数据
   + 新话题需要在udp_bridge_sender_component.h和udp_bridge_sender_component.cc中注册

   ```sh
   cyber_launch start modules/bridge/launch/bridge_sender.launch
   
   # 或点击dreamview中ApolloShenlan调试模式下的Bridge按钮
   ```

3. bridge_receiver组件

   + 用于APOLLO接收ROS的数据，需配合ROS的ros2apollo模块使用
   + 配置文件对应launch/bridge_receiver.launch、dag/bridge_receiver_89xx_xxx.dag、conf/udp_bridge_receiver_89xx_xxx.pb.txt
   + 一个dag只能接收一个数据，使用bridge_receiver.launch启动多个dag可以同时接受多个数据
   + 新话题需要在udp_bridge_receiver_component.h和udp_bridge_receiver_component.cc中注册

   ```sh
   cyber_launch start modules/bridge/launch/bridge_receiver.launch
   ```

### 1.4.注意事项

+ 若cyber_launch无法通过ctrl+c关闭，则在终端中输入`kill -9 xxx` (xxx为 cyber_launch的进程数)， 可输入 `ps -elf | grep mainboard`获取进程数

## 2.apollo_ros_bridge

+ apollo_ros_bridge是基于ROS的跨平台实时通信程序，主要包括apollo2ros模块、ros2apollo模块
  + apollo2ros模块用于ROS接收APOLLO的数据，需配合APOLLO的bridge_sender组件使用
  + apollo2ros模块配合rviz可以实现车辆位姿、点云、栅格地图、规划轨迹等数据的可视化
  
  + ros2apollo模块用于ROS向APOLLO发送数据，需配合APOLLO的bridge_receiver组件使用

### 2.1.安装protobuf

1. 复制`apollo_ros_shenlan/apollo_ros_bridge`到主目录下

2. 编译protobuf

   ```sh
   cd apollo_ros_bridge/protobuf-3.10.1
   sudo apt-get install autoconf automake libtool curl make g++ unzip
   ./autogen.sh 
   ./configure 
   make -j8
   sudo make install 
   sudo ldconfig
   
   # 编译 xxx.proto 示例：
   protoc xxx.proto --cpp_out=./
   ```

### 2.2.apollo2ros模块

+ tf为agent_0到map的动态坐标关系
+ tf_marker为车辆位姿数据，坐标系为agent_0
+ lidar为点云数据，坐标系为lidar，需配合tf和tfstatic.sh使用，其中tfstatic.sh用于发布lidar到agent_0的静态坐标关系
+ pointcloud为融合姿点云，坐标系为map
+ gridmap为占据栅格，坐标系为map
+ kino_marker为kino轨迹，坐标系为map
+ minco_marker为minco轨迹，坐标系为map

```sh
cd apollo_ros_bridge/apollo2ros
catkin_make
source devel/setup.bash
roslaunch apollo2ros xxx.launch

# 或使用脚本启动
echo "source ~/apollo_ros_shenlan/apollo_ros_bridge/apollo2ros/devel/setup.bash">> ~/.bashrc 
source ~/.bashrc
./apollo_ros_bridge/shfiles/apollo2ros/xxx.sh

# 配合rviz使用，需要修改apollo2ros/src/launch/advanced_param.xml中的地图原点坐标，
rviz -d apollo_ros_bridge/shfiles/apollo2ros.rviz
```

### 2.3.ros2apollo模块

```bash
cd apollo_ros_bridge/ros2apollo
catkin_make
source devel/setup.bash
roslaunch ros2apollo xxx.launch

# 或使用脚本启动
echo "source ~/apollo_ros_shenlan/apollo_ros_bridge/ros2apollo/devel/setup.bash">> ~/.bashrc 
source ~/.bashrc
./apollo_ros_bridge/shfiles/ros2apollo/xxx.sh
```

## 3.planner_src

+ planner_src为APOLLO中的shenlan模块的ROS源码

```bash
# 复制apollo_ros_shenlan/planner_src文件夹到主目录下

# 安装osqp
cd apollo_ros_shenlan/planner_src/osqp/build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j8
sudo make install

# 安装osqp-eigen
cd apollo_ros_shenlan/planner_src/osqp-egien/build
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
cd planner_src/src/ros-bridge
bash install_dependencies.sh

# 编译
catkin_make -DCMAKE_BUILD_TYPE=Release
# 可能报错并安装依赖
sudo apt-get install libgoogle-glog-dev

# 运行
cd planner_src/
source devel/setup.bash
roslaunch traj_planner swarm_apollo.launch
# 或使用脚本启动./planner.sh

# 将apollo数据传入planner_src需关闭仿真时间
rosparam set use_sim_time false
```

## 4.carla_apollo_bridge_guardstrikelab

+ carla_apollo_bridge_guardstrikelab用于配置CARLA仿真器

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
./apollo.sh build_opt
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
# 或使用脚本启动./9_simulator_carla.sh
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
# 或使用脚本启动./14_cyber_carla.sh

# 开启 carla bridge
# 地图参数：carla_apollo_bridge_guardstrikelab/src/carla_cyber_bridge/config/settings.yaml
cd /apollo/cyber/carla_bridge/
python carla_cyber_bridge/bridge.py
# 或使用脚本启动./bridge.sh
# ps -elf | grep carla_cyber_bridge/bridge.py

# 开启 carla spawn
# 车型参数：carla_apollo_bridge_guardstrikelab/src/carla_spawn_objects/config/objects.json
cd /apollo/cyber/carla_bridge
python carla_spawn_objects/carla_spawn_objects.py
# 或 ./vehicle.sh
# ps -elf | grep carla_spawn_objects/carla_spawn_objects.py

# 开启 carla manual
cd /apollo/cyber/carla_bridge/carla_python/examples/
python manual_control.py 
# 或使用脚本启动./manual.sh
```

### 4.5.teleop and map

```sh
# 开启 apollo teleop
./bazel-bin/moudules/canbus/tools/teleop 
# 或使用脚本启动./teleop.sh

# 配置 apollo map
# 将base_map.bin生成为sim_map.bin
./bazel-bin/modules/map/tools/sim_map_generator \
-map_dir=/apollo/modules/map/data/san_francisco/ \
-output_dir=/apollo/modules/map/data/san_francisco
# 将base_map.bin生成为routing_map.bin
./scripts/generate_routing_topo_graph.sh \
--map_dir /apollo/modules/map/data/san_francisco
```

## 5.SORA-SVL

+ SORA-SVL用于配置SVL仿真器

```sh
# 配置仿真器
git clone https://github.com/YuqiHuai/SORA-SVL.git
cp SORA-SVL/client/.env.template SORA-SVL/client/.env
cp SORA-SVL/server/.env.template SORA-SVL/server/.env
docker-compose up --build -d

# 启动 simulator
./simulator/simulator
# 点击 go offline 刷新传感器

cd SORA-SVL
docker-compose up

# 点击 go online
# 点击 open browser
# 点击 http://localhost
# 点击 API Only

# 启动 bridge
cd SORA-SVL/PythonAPI/quickstart
./01-connecting-to-simulator.py

# 启动 map & vehicle
cd SORA-SVL/examples
python 02-Apollo-v7.py
# 修改地图：sim.load("12da60a7-2fc9-474d-a62a-5cc08cb97fe8")
# 地图ID：配合SORA-SVL/server/assets/maps
# 修改车型：ego = sim.add_agent('2e966a70-4a19-44b5-a5e7-64e00a7bc5de', lgsvl.AgentType.EGO, state)
# 车型ID：SORA-SVL/server/assets/vehicles
```

