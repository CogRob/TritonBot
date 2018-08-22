load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)


cc_proto_library(
  name = "humandb_record_cc_proto",
  protos = ["humandb_record.proto"],
  proto_deps = ["//util/proto:uuid_cc_proto"],
)

py_proto_compile(
  name = "humandb_record_py_proto_only",
  protos = ["humandb_record.proto"],
  deps = ["//util/proto:uuid_py_proto_only"],
)

py_library(
  name = "humandb_record_py_proto",
  srcs = ["humandb_record_py_proto_only"],
  deps = ["//util/proto:uuid_py_proto"],
)


cc_proto_library(
  name = "humandb_service_cc_proto",
  protos = ["humandb_service.proto"],
  proto_deps = [
    ":humandb_record_cc_proto",
    "//util/proto:uuid_cc_proto",
  ],
  with_grpc = True,
)

py_proto_compile(
  name = "humandb_service_py_proto_only",
  protos = ["humandb_service.proto"],
  deps = [
    ":humandb_record_py_proto_only",
    "//util/proto:uuid_py_proto_only",
  ],
  with_grpc = True,
)

py_library(
  name = "humandb_service_py_proto",
  srcs = ["humandb_service_py_proto_only"],
  deps = [
    ":humandb_record_py_proto",
    "//util/proto:uuid_py_proto",
  ],
)
