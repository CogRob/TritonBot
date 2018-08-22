load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "smach_log_cc_proto",
  protos = ["smach_log.proto"],
)

py_proto_compile(
  name = "smach_log_py_proto_only",
  protos = ["smach_log.proto"],
)

py_library(
  name = "smach_log_py_proto",
  srcs = ["smach_log_py_proto_only"],
)
