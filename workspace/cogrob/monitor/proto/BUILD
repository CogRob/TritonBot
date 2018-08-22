load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")
load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "process_stat_cc_proto",
  protos = [
    "process_stat.proto",
  ],
  proto_deps = [
    "//util/proto:timestamp_cc_proto",
  ],
)

py_proto_compile(
  name = "process_stat_py_proto_only",
  protos = [
    "process_stat.proto",
  ],
  deps = [
    "//util/proto:timestamp_py_proto_only",
  ],
  visibility = ["//visibility:private"],
)

py_library(
  name = "process_stat_py_proto",
  srcs = ["process_stat_py_proto_only"],
  deps = [
    requirement("protobuf"),
    "//util/proto:timestamp_py_proto",
  ],
)

cc_proto_library(
  name = "log_entry_cc_proto",
  protos = [
    "log_entry.proto",
  ],
)

py_proto_compile(
  name = "log_entry_py_proto_only",
  protos = [
    "log_entry.proto",
  ],
)

py_library(
  name = "log_entry_py_proto",
  srcs = ["log_entry_py_proto_only"],
  deps = [
    requirement("protobuf"),
    "//util/proto:timestamp_py_proto",
  ],
)

cc_proto_library(
  name = "log_service_cc_proto",
  protos = [
    "log_service.proto",
  ],
  inputs = [
    "log_entry.proto",
  ],
  deps = [
    ":log_entry_cc_proto",
  ],
  with_grpc = True,
)

py_proto_compile(
  name = "log_service_py_proto_only",
  protos = [
    "log_service.proto",
  ],
  inputs = [
    "log_entry.proto",
  ],
  with_grpc = True,
)

py_library(
  name = "log_service_py_proto",
  srcs = ["log_service_py_proto_only"],
  deps = [
    requirement("protobuf"),
    ":log_entry_py_proto",
  ],
)

cc_proto_library(
  name = "docker_stats_cc_proto",
  protos = ["docker_stats.proto"],
  proto_deps = [
    "//util/proto:timestamp_cc_proto",
  ],
)

py_proto_compile(
  name = "docker_stats_py_proto_only",
  protos = ["docker_stats.proto"],
  deps = [
    "//util/proto:timestamp_py_proto_only",
  ],
)

py_library(
  name = "docker_stats_py_proto",
  srcs = ["docker_stats_py_proto_only"],
  deps = [
    requirement("protobuf"),
    "//util/proto:timestamp_py_proto",
  ],
)

cc_proto_library(
  name = "psutil_stats_cc_proto",
  protos = ["psutil_stats.proto"],
  proto_deps = [
    "//util/proto:timestamp_cc_proto",
  ],
)

py_proto_compile(
  name = "psutil_stats_py_proto_only",
  protos = ["psutil_stats.proto"],
  deps = [
    "//util/proto:timestamp_py_proto_only",
  ],
)

py_library(
  name = "psutil_stats_py_proto",
  srcs = ["psutil_stats_py_proto_only"],
  deps = [
    requirement("protobuf"),
    "//util/proto:timestamp_py_proto",
  ],
)
