set(lib_name kinematic_control)
add_library(${lib_name} SHARED
  kinematic_control.cpp
)
target_link_libraries(${lib_name}
  gz-plugin::core
  gz-sim::core
  gz-common::core
)
ament_target_dependencies(${lib_name}
  gz-plugin
  gz-sim
  gz-common
)

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
