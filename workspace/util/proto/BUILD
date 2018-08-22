load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")
load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "value_cc_proto",
  protos = ["value.proto"],
)

py_proto_compile(
  name = "value_py_proto_only",
  protos = ["value.proto"],
)

py_library(
  name = "value_py_proto",
  srcs = ["value_py_proto_only"],
  deps = [
    requirement("protobuf"),
  ],
)

cc_proto_library(
  name = "timestamp_cc_proto",
  protos = ["timestamp.proto"],
)

py_proto_compile(
  name = "timestamp_py_proto_only",
  protos = ["timestamp.proto"],
)

py_library(
  name = "timestamp_py_proto",
  srcs = ["timestamp_py_proto_only"],
  deps = [
    requirement("protobuf"),
  ],
)

cc_proto_library(
  name = "uuid_cc_proto",
  protos = ["uuid.proto"],
)

py_proto_compile(
  name = "uuid_py_proto_only",
  protos = ["uuid.proto"],
)

py_library(
  name = "uuid_py_proto",
  srcs = ["uuid_py_proto_only"],
  deps = [
    requirement("protobuf"),
  ],
)
