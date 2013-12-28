FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/adc_sub20/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/adc_sub20/msg/__init__.py"
  "../src/adc_sub20/msg/_sub20_ADC_err.py"
  "../src/adc_sub20/msg/_sub20_ADC_meas.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
