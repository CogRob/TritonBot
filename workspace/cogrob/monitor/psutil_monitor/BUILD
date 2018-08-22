load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

py_library(
  name = "psutil_monitor",
  srcs = [
    "psutil_monitor.py",
  ],
  deps = [
    requirement("absl-py"),
    requirement("psutil"),
    "//cogrob/monitor/proto:psutil_stats_py_proto",
    "//cogrob/universal_logger:universal_logger_py"
  ]
)

py_binary(
  name = "psutil_monitor_main",
  srcs = [
    "psutil_monitor_main.py",
  ],
  deps = [
    requirement("absl-py"),
    ":psutil_monitor",
  ]
)
