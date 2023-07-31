# apollo_ros_shenlan使用说明

[toc]



## 1.apollo_shenlan

### 1.1添加OMPL库

```bash
# 复制apollo_ros_shenlan/apollo_shenlan/文件夹下的内容到apollo/目录下(tools/workspace.bzl需注意替换的内容)

# 在docker内编译OMPL库
cd /apollo/third_party_shenlan/repo/ompl/build/
cmake -DCMAKE_INSTALL_PREFIX=/apollo/third_party_shenlan/ompl/ ..
make -j8
sudo make install
ls /apollo/third_party_shenlan/ompl/lib/
sudo cp -r /apollo/third_party_shenlan/ompl/lib/* /opt/apollo/sysroot/lib/
sudo ldconfig
```

### 1.2.bridge/shenlan

```bash
# 在docker内
bash apollo.sh build_opt

# bridge模块
cyber_launch start modules/bridge/launch/bridge_sender.launch
cyber_launch start modules/bridge/launch/bridge_receiver_shenlan.launch

# shenlan模块
# 开启transform、drivers、localizaiton、control模块
cyber_launch start modules/shenlan/launch/mapping.launch
cyber_launch start modules/shenlan/launch/minco.launch
```



## 2.apollo_ros_bridge

### 2.1.安装protobuf

```bash
# 复制apollo_ros_shenlan/apollo_ros_bridge文件夹到主目录下

cd apollo_ros_bridge/protobuf-master
sudo apt-get install autoconf automake libtool curl make g++ unzip
./autogen.sh 
./configure 
make -j8
sudo make install 
sudo ldconfig
```

### 2.2.apollo2ros_lidar

```bash
cd apollo_ros_bridge/apollo2ros_lidar
catkin_make
roscore
source devel/setup.bash
rosrun lidar lidar
```

### 2.3.ros2apollo_trajectory

```bash
cd apollo_ros_bridge/ros2apollo_trajectory
catkin_make
roscore
source devel/setup.bash
rosrun trajectory trajectory
```



## 3.car_src

```bash
# 复制apollo_ros_shenlan/carla文件夹到主目录下

# 安装osqp
cd apollo_ros_shenlan/carla/osqp/build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j8
sudo make install

#安装osqp-eigen
cd apollo_ros_shenlan/carla/osqp-egien/build
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
cd carla/car_src/src/ros-bridge
bash install_dependencies.sh

# 编译
catkin_make -DCMAKE_BUILD_TYPE=Release
# 可能报错并安装依赖
sudo apt-get install libgoogle-glog-dev

# 运行
cd carla/car_src/
source devel/setup.bash
roslaunch traj_planner swarm_apollo.launch
```



## 4.注意事项

1.若cyber_launch无法通过ctrl+c关闭，则在终端中输入kill -9 xxx (xxx为 cyber_launch的进程数字)

2.编译proto：protoc addressbook.proto --cpp_out=./

3.rosparam set use_sim_time false

