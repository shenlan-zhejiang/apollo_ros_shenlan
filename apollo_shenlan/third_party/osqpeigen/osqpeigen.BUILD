load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# TODO(all): May use rules_boost.
cc_library(
    name = "osqpeigen",
    includes = [".",],
    hdrs = glob([
        "OsqpEigen/*.hpp",
        "OsqpEigen/*.h",
    ]), 
    deps = [
    	"@osqp2",
    ],
)
