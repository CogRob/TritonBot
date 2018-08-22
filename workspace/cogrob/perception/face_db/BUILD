load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

py_library(
  name = "facedb_storage",
  srcs = [
    "facedb_storage.py",
  ],
  deps = [
    requirement("absl-py"),
    requirement("pandas"),
    "//cogrob/perception/face_db/proto:storage_py_proto",
    "//util:proto_file_util",
    "//util:uuid_util",
  ],
);

py_binary(
  name = "facedb_server",
  srcs = [
    "facedb_server.py",
    "facedb_storage.py",
  ],
  deps = [
    requirement("absl-py"),
    requirement("pandas"),
    "//cogrob/perception/face_db/proto:storage_py_proto",
    "//cogrob/perception/face_db/proto:facedb_service_py_proto",
    "//util:proto_file_util",
    "//util:uuid_util",
    "//util:wait_to_terminate",
  ],
);

py_test(
  name = "facedb_server_test",
  srcs = [
    "facedb_server_test.py",
  ],
  deps = [
    ":facedb_server",
    "//util:uuid_util",
  ],
  data = [
    "//cogrob/perception/face_db:testdata"
  ]
);
