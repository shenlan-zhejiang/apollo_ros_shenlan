load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
minco_mapping_COPTS = ['-DMODULE_NAME=\\"minco_mapping\\" -Wno-unused-but-set-variable -std=c++14 -g']
cc_library(
    name="minco_mapping", 
    srcs=["kino_astar.cpp","raycast.cpp","traj_optimizer.cpp", "traj_manager.cpp", "replan_fsm.cpp", "minco_mapping_component.cc"],
    copts = minco_mapping_COPTS,
    hdrs=glob(["geo_utils2d/*.hpp",
    "plan_utils/*.hpp",
    "path_searching/*.h",
    "plan_manage/*.h",
    "*.h"]),
    deps=[
        "//cyber",
        "//modules/drivers/lidar/proto:velodyne_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
        "//modules/shenlan/mapping",
        "//modules/shenlan/mpc/proto:EgoVehicleControl_cc_proto",
        "//modules/shenlan/mpc/proto:EgoVehicleInfo_cc_proto",
        "//modules/shenlan/mpc/proto:EgoVehicleStatus_cc_proto",
        "//modules/shenlan/mpc/proto:Trajectory_cc_proto",
        "//modules/shenlan/proto:shenlan_pb_cc_proto",
        "@eigen",
        "@boost",
        "@local_config_pcl//:pcl",
        "@ompl",
        "@com_github_google_glog//:glog",
        "@com_github_gflags_gflags//:gflags",
        "@osqp2",
        "@osqpeigen",
    ],
)

cc_binary(
   name="libminco_mapping_component.so",
   linkshared=True,
   linkstatic=False,
   deps=[":minco_mapping"],
)

cpplint()