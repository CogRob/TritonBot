load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

py_library(
  name = "humandb_model",
  srcs = [
    "humandb_model.py",
  ],
  deps = [
    requirement("absl-py"),
    requirement("pandas"),
    "//cogrob/identity/human_db/proto:humandb_record_py_proto",
    "//util:proto_file_util",
    "//util:uuid_util",
  ],
);

py_binary(
  name = "humandb_server",
  srcs = [
    "humandb_server.py",
  ],
  deps = [
    ":humandb_model",
    requirement("pandas"),
    requirement("grpcio"),
    "//cogrob/identity/human_db/proto:humandb_record_py_proto",
    "//cogrob/identity/human_db/proto:humandb_service_py_proto",
    "//util:wait_to_terminate",
  ],
);

py_test(
  name = "humandb_model_test",
  srcs = [
    "humandb_model_test.py",
  ],
  deps = [
    ":humandb_model",
    "//util:proto_file_util",
    "//util:uuid_util",
  ],
);
