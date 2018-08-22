package(
  default_visibility = ["//cogrob/dialogue:__subpackages__"],
)

py_library(
  name = "speech_interface",
  srcs = [
    "speech_interface.py",
  ],
)

py_library(
  name = "mock_speech_recognizer",
  srcs = [
    "mock_speech_recognizer.py",
  ],
  deps = [
    ":speech_interface",
  ],
)

py_test(
  name = "mock_speech_recognizer_test",
  srcs = [
    "mock_speech_recognizer_test.py",
  ],
  deps = [
    ":mock_speech_recognizer",
  ],
)
