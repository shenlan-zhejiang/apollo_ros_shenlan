load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name="mpc", 
    srcs=["cubic_spline_planner.cpp","mpc.cpp", "mpc_component.cc"],
    hdrs=["common/cubic_spline_planner.h","common/traj_anal.hpp", "minco_util/root_finder.hpp", "minco_util/poly_traj_utils.hpp","mpc.h", "mpc_component.h"],
    deps=[
        "//cyber",
        "//modules/drivers/lidar/lidar_robosense/proto:suteng_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/shenlan/mpc/proto:EgoVehicleControl_cc_proto",
        "//modules/shenlan/mpc/proto:EgoVehicleInfo_cc_proto",
        "//modules/shenlan/mpc/proto:EgoVehicleStatus_cc_proto",
        "//modules/shenlan/mpc/proto:Trajectory_cc_proto",
        "@eigen",
        "@osqp2",
        "@osqpeigen",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
   name="libmpc_component.so",
   linkshared=True,
   linkstatic=False,
   deps=[":mpc"],
)

cpplint()
