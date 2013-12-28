FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/bmc050_driver/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/bmc050_measurement.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_bmc050_measurement.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
