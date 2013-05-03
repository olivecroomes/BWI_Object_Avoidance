FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/objectAvoidance/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/objectAvoidance/msg/__init__.py"
  "../src/objectAvoidance/msg/_Sonar.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
