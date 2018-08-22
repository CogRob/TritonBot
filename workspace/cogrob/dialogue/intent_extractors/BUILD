load("@pip_deps//:requirements.bzl", "requirement")

package(
  default_visibility = ["//cogrob/dialogue:__subpackages__"],
)

py_library(
  name = "extractor_base",
  srcs = [
    "extractor_base.py",
  ],
  deps = [
    requirement("protobuf"),
    requirement("absl-py"),
    "//cogrob/dialogue/intent:intent_py_proto",
  ],
)


py_library(
  name = "intent_extractors",
  srcs = glob(["*_extractor.py"]),
  deps = [
    ":extractor_base",
    "//cogrob/dialogue/intent_util:lcs_matcher",
    "//cogrob/dialogue/intent:intent_py_proto",
    "//cogrob/dialogue/quiz:all_quiz",
    "//cogrob/navigation/tour:tour_info",
  ],
)

py_test(
  name = "yes_no_extractor_test",
  srcs = [
    "yes_no_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "how_are_you_extractor_test",
  srcs = [
    "how_are_you_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "number_extractor_test",
  srcs = [
    "number_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "action_extractor_test",
  srcs = [
    "action_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "general_text_extractor_test",
  srcs = [
    "general_text_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "user_question_extractor_test",
  srcs = [
    "user_question_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "name_extractor_test",
  srcs = [
    "name_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "quiz_extractor_test",
  srcs = [
    "quiz_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "combined_extractor_test",
  srcs = [
    "combined_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)

py_test(
  name = "tour_extractor_test",
  srcs = [
    "tour_extractor_test.py",
  ],
  deps = [
    ":intent_extractors",
  ]
)
