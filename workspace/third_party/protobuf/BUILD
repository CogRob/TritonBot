licenses(["notice"])

package(
  default_visibility = ["//visibility:public"],
)

cc_library(
  name = "protobuf",
  deps = [
    "@com_google_protobuf//:protobuf",
    ":protobuf_gzip_stream_zlibfix",
  ],
  visibility = ["//visibility:public"],
);

cc_library(
  name = "protobuf_gzip_stream_zlibfix",
  srcs = ["gzip_stream.cc"],
  deps = ["@com_google_protobuf//:protobuf"],
  visibility = ["//visibility:public"],
  copts = ["-DHAVE_ZLIB"],
)
