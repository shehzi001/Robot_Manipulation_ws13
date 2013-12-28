FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/bmp085/msg"
  "../src/bmp085/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/bmp085/msg/__init__.py"
  "../src/bmp085/msg/_bmp085_measurement.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
