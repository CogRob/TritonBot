load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

cc_library(
  name = "universal_logger",
  srcs = ["universal_logger.cc"],
  hdrs = [
    "universal_logger.h",
    "universal_logger_interface.h",
    "universal_logger_flusher_interface.h",
  ],
  deps = [
    "//cogrob/universal_logger/proto:archive_entry_cc_proto",
    "//cogrob/universal_logger/proto:archive_header_cc_proto",
    "//third_party:gflags",
    "//third_party:glog",
    "//third_party/protobuf",
    "//util:file_system",
    "//util:status",
    "//util:timestamp",
    "//util:uuid",
  ],
)

cc_library(
  name = "mock_universal_logger",
  hdrs = [
    "universal_logger_interface.h",
    "universal_logger_flusher_interface.h",
    "universal_logger_mock.h",
  ],
  deps = [
    "//cogrob/universal_logger/proto:archive_entry_cc_proto",
    "//cogrob/universal_logger/proto:archive_header_cc_proto",
    "//third_party:gflags",
    "//third_party:glog",
    "//third_party/protobuf",
    "//util:file_system",
    "//util:status",
    "//util:timestamp",
    "//util:uuid",
  ],
)

cc_test(
  name = "universal_logger_test",
  srcs = [
    "universal_logger_test.cc",
  ],
  deps = [
    ":universal_logger",
    "//cogrob/universal_logger/proto:archive_entry_cc_proto",
    "//third_party:gtest_main",
    "//third_party:glog",
  ],
)

cc_library(
  name = "universal_logger_flusher",
  srcs = ["universal_logger_flusher.cc"],
  hdrs = [
    "universal_logger_flusher.h",
    "universal_logger_flusher_interface.h",
  ],
  deps = [
    ":universal_logger",
    "//cogrob/universal_logger/proto:archive_entry_cc_proto",
    "//cogrob/universal_logger/proto:archive_header_cc_proto",
    "//third_party/absl:synchronization",
    "//third_party/absl:time",
    "//third_party:gflags",
    "//third_party:glog",
    "//util:status",
  ],
)

cc_test(
  name = "universal_logger_flusher_test",
  srcs = [
    "universal_logger_flusher_test.cc",
    "universal_logger_interface.h",
  ],
  deps = [
    ":mock_universal_logger",
    ":universal_logger_flusher",
    "//third_party:gtest_main",
    "//third_party:glog",
    "//util:status",
  ],
)

py_library(
  name = "universal_logger_py",
  srcs = ["universal_logger.py"],
  deps = [
    requirement("absl-py"),
    requirement("protobuf"),
    "//cogrob/universal_logger/proto:archive_entry_py_proto",
    "//cogrob/universal_logger/proto:archive_header_py_proto",
  ],
)

py_binary(
  name = "universal_log_reader_py",
  main = "universal_log_reader.py",
  srcs = ["universal_log_reader.py"],
  deps = [
    requirement("absl-py"),
    requirement("protobuf"),
    "//cogrob/universal_logger/proto:archive_entry_py_proto",
    "//cogrob/universal_logger/proto:archive_header_py_proto",
  ],
)

py_test(
  name = "universal_logger_py_test",
  main = "universal_logger_test.py",
  srcs = ["universal_logger_test.py"],
  deps = [
    ":universal_logger_py",
  ],
)

cc_library(
  name = "event_entry_helper",
  srcs = [
    "event_entry_helper.cc",
  ],
  hdrs = [
    "event_entry_helper.h",
    "event_entry_helper_interface.h",
  ],
  deps = [
    ":universal_logger",
    "//cogrob/universal_logger/proto:archive_entry_cc_proto",
    "//third_party/absl:memory",
    "//util:timestamp",
  ],
)

cc_test(
  name = "event_entry_helper_test",
  srcs = [
    "event_entry_helper_test.cc",
  ],
  deps = [
    ":event_entry_helper",
    ":mock_universal_logger",
    "//third_party:gtest_main",
    "//third_party:glog",
  ],
)
