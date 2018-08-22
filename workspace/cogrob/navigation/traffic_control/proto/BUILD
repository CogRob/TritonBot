load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "traffic_control_cc_proto",
  protos = ["traffic_control.proto"],
  proto_deps = ["//cogrob/navigation/proto:navigation_chart_cc_proto"],
  with_grpc = True,
)

py_proto_compile(
  name = "traffic_control_py_proto_only",
  protos = ["traffic_control.proto"],
  deps = ["//cogrob/navigation/proto:navigation_chart_py_proto_only"],
  with_grpc = True,
)

py_library(
  name = "traffic_control_py_proto",
  srcs = ["traffic_control_py_proto_only"],
  deps = ["//cogrob/navigation/proto:navigation_chart_py_proto"],
)
