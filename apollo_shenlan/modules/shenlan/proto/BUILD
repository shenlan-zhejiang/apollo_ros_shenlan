load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "shenlan_pb_cc_proto",
    deps = [
        ":shenlan_pb_proto",
    ],
)

proto_library(
    name = "shenlan_pb_proto",
    srcs = ["shenlan_pb.proto"],
    deps = [
        "//modules/common/proto:header_proto",
        "//modules/localization/proto:pose_proto",
    ],
)