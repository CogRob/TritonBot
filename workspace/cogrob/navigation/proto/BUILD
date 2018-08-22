load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")
load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "occupancy_grid_cc_proto",
  protos = ["occupancy_grid.proto"],
)

py_proto_compile(
  name = "occupancy_grid_py_proto_only",
  protos = ["occupancy_grid.proto"],
)

py_library(
  name = "occupancy_grid_py_proto",
  srcs = ["occupancy_grid_py_proto_only"],
  deps = [
    requirement("protobuf"),
  ],
)

cc_proto_library(
  name = "navigation_chart_cc_proto",
  protos = ["navigation_chart.proto"],
  proto_deps = [":occupancy_grid_cc_proto"],
)

py_proto_compile(
  name = "navigation_chart_py_proto_only",
  protos = ["navigation_chart.proto"],
  deps = [":occupancy_grid_py_proto_only"],
)

py_library(
  name = "navigation_chart_py_proto",
  srcs = ["navigation_chart_py_proto_only"],
  deps = [
    requirement("protobuf"),
    ":occupancy_grid_py_proto",
  ],
)

cc_proto_library(
  name = "logging_cc_proto",
  protos = ["logging.proto"],
  proto_deps = [
    ":navigation_chart_cc_proto",
    "//util/proto:timestamp_cc_proto",
  ],
)

py_proto_compile(
  name = "logging_py_proto_only",
  protos = ["logging.proto"],
  deps = [
    ":navigation_chart_py_proto_only",
    "//util/proto:timestamp_py_proto_only",
  ],
)

py_library(
  name = "logging_py_proto",
  srcs = ["logging_py_proto_only"],
  deps = [
    requirement("protobuf"),
    ":navigation_chart_py_proto",
    "//util/proto:timestamp_py_proto",
  ],
)
