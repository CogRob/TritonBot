package(
  default_visibility = ["//cogrob/dialogue:__subpackages__"],
)

py_library(
  name = "intent_selector_base",
  srcs = [
    "intent_selector_base.py",
  ],
  deps = [
    "//cogrob/dialogue/intent:intent_py_proto",
    "//cogrob/dialogue/speech_recognition:speech_interface",
  ],
)

py_library(
  name = "timed_best_intent_selector",
  srcs = [
    "timed_best_intent_selector.py",
  ],
  deps = [
    ":intent_selector_base",
    "//cogrob/dialogue/intent:intent_py_proto",
    "//cogrob/dialogue/intent_extractors:intent_extractors",
  ],
)

py_test(
  name = "timed_best_intent_selector_test",
  srcs = [
    "timed_best_intent_selector_test.py",
  ],
  deps = [
    ":timed_best_intent_selector",
    "//cogrob/dialogue/intent:intent_py_proto",
    "//cogrob/dialogue/intent_extractors:intent_extractors",
  ]
)
