FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/amtec/msg"
  "../src/amtec/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/amtec/msg/__init__.py"
  "../src/amtec/msg/_AmtecState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
