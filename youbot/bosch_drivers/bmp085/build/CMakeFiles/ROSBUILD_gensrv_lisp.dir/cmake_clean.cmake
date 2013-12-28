FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/bmp085/msg"
  "../src/bmp085/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/weather_forcast.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_weather_forcast.lisp"
  "../srv_gen/lisp/measure.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_measure.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
