load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

package(
  default_visibility = ["//visibility:public"],
)


cc_proto_library(
  name = "facedb_service_cc_proto",
  protos = ["facedb_service.proto"],
  proto_deps = ["//util/proto:uuid_cc_proto"],
  with_grpc = True,
)

py_proto_compile(
  name = "facedb_service_py_proto_only",
  protos = ["facedb_service.proto"],
  deps = ["//util/proto:uuid_py_proto_only"],
  with_grpc = True,
)

py_library(
  name = "facedb_service_py_proto",
  srcs = ["facedb_service_py_proto_only"],
  deps = ["//util/proto:uuid_py_proto"],
)


cc_proto_library(
  name = "storage_cc_proto",
  protos = ["storage.proto"],
  proto_deps = ["//util/proto:uuid_cc_proto"],
)

py_proto_compile(
  name = "storage_py_proto_only",
  protos = ["storage.proto"],
  deps = ["//util/proto:uuid_py_proto_only"],
)

py_library(
  name = "storage_py_proto",
  srcs = ["storage_py_proto_only"],
  deps = ["//util/proto:uuid_py_proto"],
)
