load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)

cc_proto_library(
  name = "intent_cc_proto",
  protos = glob(["*.proto"]),
  proto_deps = [
    "//util/proto:timestamp_cc_proto",
  ],
)

py_proto_compile(
  name = "intent_py_proto_only",
  protos = glob(["*.proto"]),
  deps = [
    "//util/proto:timestamp_py_proto_only",
  ],
)

py_library(
  name = "intent_py_proto",
  srcs = [
    ":intent_py_proto_only"
  ],
  deps = [
    "//util/proto:timestamp_py_proto",
  ],
)
