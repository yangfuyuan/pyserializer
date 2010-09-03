FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv/cpp/serializer/TravelDistance.h"
  "../srv/cpp/serializer/GetAnalog.h"
  "../srv/cpp/serializer/GetDigital.h"
  "../srv/cpp/serializer/SetDigital.h"
  "../srv/cpp/serializer/IsMoving.h"
  "../srv/cpp/serializer/RotateAngle.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
