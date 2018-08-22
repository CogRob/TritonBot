package(
  default_visibility = ["//cogrob/dialogue:__subpackages__"],
)

py_library(
  name = "intent_listener",
  srcs = [
    "intent_listener.py",
  ],
  deps = [
    "//cogrob/dialogue/intent_selector:intent_selector_base",
    "//cogrob/dialogue/speech_recognition:speech_interface",
  ],
)

py_test(
  name = "intent_listener_test",
  srcs = [
    "intent_listener_test.py",
  ],
  deps = [
    ":intent_listener",
    "//cogrob/dialogue/intent_extractors",
    "//cogrob/dialogue/intent_selector:timed_best_intent_selector",
    "//cogrob/dialogue/speech_recognition:mock_speech_recognizer",
  ],
)
