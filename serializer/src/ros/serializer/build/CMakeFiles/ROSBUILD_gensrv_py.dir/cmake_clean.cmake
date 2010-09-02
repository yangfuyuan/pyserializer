FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/serializer/srv/__init__.py"
  "../src/serializer/srv/_IsMoving.py"
  "../src/serializer/srv/_SetDigital.py"
  "../src/serializer/srv/_GetAnalog.py"
  "../src/serializer/srv/_GetDigital.py"
  "../src/serializer/srv/_TravelDistance.py"
  "../src/serializer/srv/_RotateAngle.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
