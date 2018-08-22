load("@pip_deps//:requirements.bzl", "requirement")

cc_library(
  name = "traffic_control_server",
  srcs = ["traffic_control_server.cc"],
  hdrs = ["traffic_control_server.h"],
  deps = [
    "//cogrob/navigation/traffic_control/proto:traffic_control_cc_proto",
    "//third_party/absl:synchronization",
    "//third_party:glog",
    "//util:hash",
  ],
);

cc_binary(
  name = "traffic_control_server_main",
  srcs = ["traffic_control_server_main.cc"],
  deps = [
    ":traffic_control_server",
    "//third_party/absl:memory",
    "//third_party:gflags",
    "//third_party:glog",
  ],
);

py_binary(
  name = "traffic_control_example_client",
  srcs = ["traffic_control_example_client.py"],
  deps = [
    requirement("absl-py"),
    requirement("grpcio"),
    "//cogrob/navigation/traffic_control/proto:traffic_control_py_proto",
  ],
);
