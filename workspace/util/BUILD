package(
  default_visibility = ["//visibility:public"],
)


cc_library(
  name = "status",
  srcs = [
    "status.h",
    "status.cc",
    "statusor.h",
    "statusor.cc",
    "status_macros.h",
  ],
  hdrs = [
    "status.h",
    "statusor.h",
    "status_macros.h"
  ],
  deps = [
    "//third_party:glog",
  ],
)

cc_test(
  name = "status_test",
  srcs = [
    "status_test.cc",
  ],
  deps = [
    ":status",
    "//third_party:gtest_main",
  ],
)

cc_test(
  name = "statusor_test",
  srcs = [
    "statusor_test.cc",
  ],
  deps = [
    ":status",
    "//third_party:gtest_main",
  ],
)


cc_library(
  name = "ptr_util",
  hdrs = ["ptr_util.h"],
)

cc_test(
  name = "ptr_util_test",
  srcs = [
    "ptr_util_test.cc",
  ],
  deps = [
    ":ptr_util",
    "//third_party:gtest_main",
  ],
)


cc_library(
  name = "simple_thread_safe_queue",
  hdrs = ["simple_thread_safe_queue.h"],
  deps = [
    ":status",
  ],
)

cc_test(
  name = "simple_thread_safe_queue_test",
  srcs = [
    "simple_thread_safe_queue_test.cc",
  ],
  deps = [
    ":simple_thread_safe_queue",
    "//third_party:gtest_main",
  ],
)


cc_library(
  name = "size_limit_map",
  hdrs = ["size_limit_map.h"],
  deps = [
    ":status",
  ],
)

cc_test(
  name = "size_limit_map_test",
  srcs = [
    "size_limit_map_test.cc",
  ],
  deps = [
    ":size_limit_map",
    "//third_party:gtest_main",
  ],
)


cc_library(
  name = "hash",
  hdrs = ["hash.h"],
)


cc_library(
  name = "uuid",
  srcs = ["uuid.cc"],
  hdrs = ["uuid.h"],
  linkopts = ["-luuid"],
  deps = [
    "//util/proto:uuid_cc_proto"
  ]
)

cc_test(
  name = "uuid_test",
  srcs = [
    "uuid_test.cc",
  ],
  deps = [
    ":uuid",
    "//util/proto:uuid_cc_proto",
    "//third_party:gtest_main",
    "//third_party:glog",
  ],
)


cc_library(
  name = "file_system",
  srcs = ["file_system.cc"],
  hdrs = ["file_system.h"],
  deps = [
    ":status",
  ]
)

cc_test(
  name = "file_system_test",
  srcs = [
    "file_system_test.cc",
  ],
  deps = [
    ":file_system",
    "//third_party:gtest_main",
    "//third_party:glog",
  ],
)


cc_library(
  name = "file_content",
  srcs = ["file_content.cc"],
  hdrs = ["file_content.h"],
  deps = [
    ":status",
  ]
)

cc_test(
  name = "file_content_test",
  srcs = [
    "file_content_test.cc",
  ],
  deps = [
    ":file_content",
    "//third_party:gtest_main",
    "//third_party:glog",
  ],
  data = [
    "//util:testdata",
  ],
)


cc_library(
  name = "timestamp",
  srcs = ["timestamp.cc"],
  hdrs = ["timestamp.h"],
  deps = [
    "//util/proto:timestamp_cc_proto",
  ]
)

cc_test(
  name = "timestamp_test",
  srcs = [
    "timestamp_test.cc",
  ],
  deps = [
    ":timestamp",
    "//third_party:gtest_main",
    "//third_party:glog",
    "//util/proto:timestamp_cc_proto",
  ],
)

py_library(
  name = "wait_to_terminate",
  srcs = ["wait_to_terminate.py"],
)

py_library(
  name = "uuid_util",
  srcs = ["uuid_util.py"],
)

py_library(
  name = "proto_file_util",
  srcs = ["proto_file_util.py"],
)

py_library(
  name = "protobuf_util",
  srcs = ["protobuf_util.py"],
)
