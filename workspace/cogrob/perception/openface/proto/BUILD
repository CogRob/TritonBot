load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "common_cc_proto",
  protos = ["common.proto"],
)

py_proto_compile(
  name = "common_py_proto_only",
  protos = ["common.proto"],
)

py_library(
  name = "common_py_proto",
  srcs = ["common_py_proto_only"],
)


cc_proto_library(
  name = "openface_service_cc_proto",
  protos = ["openface_service.proto"],
  proto_deps = [":common_cc_proto"],
  with_grpc = True,
)

py_proto_compile(
  name = "openface_service_py_proto_only",
  protos = ["openface_service.proto"],
  deps = [":common_py_proto_only"],
  with_grpc = True,
)

py_library(
  name = "openface_service_py_proto",
  srcs = ["openface_service_py_proto_only"],
  deps = [":common_py_proto"],
)
