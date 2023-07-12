load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

SHENLAN_COPTS = ['-DMODULE_NAME=\\"shenlan\\"']

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_library(
    name = "mapping",
    copts = SHENLAN_COPTS,
    srcs = ["mapping.cpp", "raycast.cpp", "mapping_component.cc"],
    hdrs = ["mapping.h", "raycast.h", "mapping_component.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/drivers/lidar/lidar_robosense/proto:suteng_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
        "//modules/shenlan/proto:shenlan_conf_cc_proto",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_gflags_gflags//:gflags"
    ],
)

cc_binary(
    name = "libmapping_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":mapping"],
)

cpplint()
