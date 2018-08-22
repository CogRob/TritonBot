licenses(["notice"])

load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

cc_library(
  name = "gflags",
  hdrs = ["gflags.h"],
  deps = ["@com_github_gflags_gflags//:gflags"],
  linkopts = ["-pthread"],
)

cc_library(
  name = "glog",
  hdrs = ["glog.h"],
  deps = ["@com_github_google_glog//:glog"],
)

cc_library(
  name = "gtest_main",
  hdrs = ["gtest.h", "gmock.h"],
  deps = ["@com_google_googletest//:gtest_main"],
  testonly = True,
)

cc_library(
  name = "gtest",
  hdrs = ["gtest.h", "gmock.h"],
  deps = ["@com_google_googletest//:gtest"],
)

cc_library(
  name = "libportaudio",
  hdrs = ["portaudio.h"],
  # deps = ["@portaudio_tgz//:libportaudio"],
  linkopts = ["-lportaudio"],
)

cc_library(
  name = "libsnowboy",
  deps = ["@snowboy_git//:libsnowboy"],
  visibility = ["//cogrob/speech/wakeword:__pkg__"],
)

alias(
  name = "google_cloud_speech",
  actual = "@googleapis_git//:cloud_speech",
)

alias(
  name = "google_datastore",
  actual = "@googleapis_git//:datastore",
)

cc_library(
  name = "pqxx",
  hdrs = ["pqxx.h"],
  linkopts = ["-lpqxx"],
)

cc_library(
  name = "hashlibpp",
  hdrs = ["hashlibpp.h"],
  deps = ["@hashlibpp_zip//:hashlib2plus"],
)

py_library(
  name = "use_ros_python",
  srcs = [
    "use_ros_python.py",
  ],
  deps = [
    requirement("pyros_setup"),
  ],
  data = [
    "pyros_setup.cfg",
  ],
)

py_library(
  name = "python_protobuf",
  deps = [
    requirement("protobuf"),
  ],
)

py_library(
  name = "python_grpc",
  deps = [
    requirement("enum34"),
    requirement("grpcio"),
    requirement("protobuf"),
  ],
)

py_library(
  name = "python_absl",
  deps = [
    requirement("absl-py"),
  ],
)
