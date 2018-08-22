load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//visibility:public"],
)

py_library(
  name = "docker_monitor",
  srcs = [
    "docker_monitor.py",
  ],
  deps = [
    requirement("absl-py"),
    requirement("backports.ssl_match_hostname"),
    requirement("docker"),
    requirement("docker-pycreds"),
    requirement("ipaddress"),
    requirement("requests"),
    requirement("six"),
    requirement("websocket"),
    requirement("python-dateutil"),
    "//cogrob/monitor/proto:docker_stats_py_proto",
    "//cogrob/universal_logger:universal_logger_py"
  ]
)

py_binary(
  name = "docker_monitor_main",
  srcs = [
    "docker_monitor_main.py",
  ],
  deps = [
    requirement("absl-py"),
    ":docker_monitor",
  ]
)
