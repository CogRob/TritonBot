load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "voice_recognition_log_cc_proto",
  protos = ["voice_recognition_log.proto"],
  proto_deps = [
    "//util/proto:timestamp_cc_proto",
  ],
)

py_proto_compile(
  name = "voice_recognition_log_py_proto_only",
  protos = ["voice_recognition_log.proto"],
  deps = [
    "//util/proto:timestamp_py_proto_only",
  ],
)

py_library(
  name = "voice_recognition_log_py_proto",
  srcs = ["voice_recognition_log_py_proto_only"],
  deps = [
    "//util/proto:timestamp_py_proto",
  ],
)


cc_proto_library(
  name = "robot_speech_log_cc_proto",
  protos = ["robot_speech_log.proto"],
  proto_deps = [
    "//util/proto:timestamp_cc_proto",
  ],
)

py_proto_compile(
  name = "robot_speech_log_py_proto_only",
  protos = ["robot_speech_log.proto"],
  deps = [
    "//util/proto:timestamp_py_proto_only",
  ],
)

py_library(
  name = "robot_speech_log_py_proto",
  srcs = ["robot_speech_log_py_proto_only"],
  deps = [
    "//util/proto:timestamp_py_proto",
  ],
)
