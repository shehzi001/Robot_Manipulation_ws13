FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/amtec/msg"
  "../src/amtec/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/amtec/srv/__init__.py"
  "../src/amtec/srv/_TargetAcceleration.py"
  "../src/amtec/srv/_Reset.py"
  "../src/amtec/srv/_GetStatus.py"
  "../src/amtec/srv/_SweepTilt.py"
  "../src/amtec/srv/_Home.py"
  "../src/amtec/srv/_SetVelocity.py"
  "../src/amtec/srv/_TargetVelocity.py"
  "../src/amtec/srv/_Halt.py"
  "../src/amtec/srv/_SweepPan.py"
  "../src/amtec/srv/_SetPosition.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
