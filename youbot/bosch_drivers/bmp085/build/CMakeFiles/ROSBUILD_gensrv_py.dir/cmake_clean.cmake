FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/bmp085/msg"
  "../src/bmp085/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/bmp085/srv/__init__.py"
  "../src/bmp085/srv/_weather_forcast.py"
  "../src/bmp085/srv/_measure.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
