In the abseil-cpp repoistory, create a fake :anything target, then:
```
bazel query "visible(//:anything, kind(cc_library, //absl/...))" | sort
```

Will need manually remove testing and internal targets.
