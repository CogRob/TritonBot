# `use_cogrob_workspace`

Allows using Python code in workspace with ROS.

First, set `BAZEL_WORKSPACE` environment variable. Then run `link_from_bazel.sh`
script. You may need `pip install grpcio-tools` for this. In any Python file
you need to use anything from `workspace`, do `import use_cogrob_workspace`.

If you create/remove any file in `workspace`, rerun the sync script.
