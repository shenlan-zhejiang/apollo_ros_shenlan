load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "shenlan_conf_cc_proto",
    deps = [
        ":shenlan_conf_proto",
    ],
)

proto_library(
    name = "shenlan_conf_proto",
    srcs = ["shenlan_conf.proto"],
)

py_proto_library(
    name = "shenlan_conf_py_pb2",
    deps = [
        ":shenlan_conf_proto",
    ],
)