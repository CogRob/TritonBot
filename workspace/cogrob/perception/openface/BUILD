load("@pip_deps//:requirements.bzl", "requirement")

py_binary(
  name = "openface_server",
  srcs = [
    "openface_server.py",
    "//cogrob/perception/openface/proto:common_py_proto",
    "//cogrob/perception/openface/proto:openface_service_py_proto",
  ],
  deps = [
    requirement("absl-py"),
    "//cogrob/universal_logger/proto:archive_entry_py_proto",
    "//cogrob/universal_logger:universal_logger_py",
  ],
);
