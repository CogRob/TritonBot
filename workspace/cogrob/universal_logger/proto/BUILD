load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "archive_header_cc_proto",
  protos = ["archive_header.proto"],
)

py_proto_compile(
  name = "archive_header_py_proto_only",
  protos = ["archive_header.proto"],
)

py_library(
  name = "archive_header_py_proto",
  srcs = ["archive_header_py_proto_only"],
)

cc_proto_library(
  name = "archive_entry_cc_proto",
  protos = ["archive_entry.proto"],
  proto_deps = [
    "//util/proto:timestamp_cc_proto",
    "//util/proto:uuid_cc_proto",
    "//cogrob/dialogue/logging/proto:robot_speech_log_cc_proto",
    "//cogrob/dialogue/logging/proto:voice_recognition_log_cc_proto",
    "//cogrob/monitor/proto:docker_stats_cc_proto",
    "//cogrob/monitor/proto:psutil_stats_cc_proto",
    "//cogrob/navigation/proto:logging_cc_proto",
    "//cogrob/perception/openface/proto:common_cc_proto",
    "//cogrob/executive/proto:smach_log_cc_proto",
  ],
)

py_proto_compile(
  name = "archive_entry_py_proto_only",
  protos = ["archive_entry.proto"],
  deps = [
    "//util/proto:timestamp_py_proto_only",
    "//util/proto:uuid_py_proto_only",
    "//cogrob/dialogue/logging/proto:robot_speech_log_py_proto_only",
    "//cogrob/dialogue/logging/proto:voice_recognition_log_py_proto_only",
    "//cogrob/monitor/proto:docker_stats_py_proto_only",
    "//cogrob/monitor/proto:psutil_stats_py_proto_only",
    "//cogrob/navigation/proto:logging_py_proto_only",
    "//cogrob/perception/openface/proto:common_py_proto_only",
    "//cogrob/executive/proto:smach_log_py_proto_only",
  ],
)

py_library(
  name = "archive_entry_py_proto",
  srcs = ["archive_entry_py_proto_only"],
  deps = [
    "//util/proto:timestamp_py_proto",
    "//util/proto:uuid_py_proto",
    "//cogrob/perception/openface/proto:common_py_proto",
    "//cogrob/dialogue/logging/proto:robot_speech_log_py_proto",
    "//cogrob/monitor/proto:docker_stats_py_proto",
    "//cogrob/monitor/proto:psutil_stats_py_proto",
    "//cogrob/navigation/proto:logging_py_proto",
    "//cogrob/dialogue/logging/proto:voice_recognition_log_py_proto",
    "//cogrob/executive/proto:smach_log_py_proto",
  ],
)
