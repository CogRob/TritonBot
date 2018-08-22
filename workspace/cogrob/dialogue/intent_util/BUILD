package(default_visibility = ["//cogrob/dialogue:__subpackages__"])

py_library(
  name = "lcs_matcher",
  srcs = ["lcs_matcher.py"],
)

py_test(
  name = "lcs_matcher_test",
  srcs = [
    "lcs_matcher_test.py",
  ],
  deps = [
    ":lcs_matcher",
  ],
)
