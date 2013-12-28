FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/bmc050_driver/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/bmc050_driver/msg/__init__.py"
  "../src/bmc050_driver/msg/_bmc050_measurement.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
