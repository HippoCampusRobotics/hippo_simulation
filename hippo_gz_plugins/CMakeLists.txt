cmake_minimum_required(VERSION 3.5)
project(hippo_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# needed for clang-tidy to work
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2)
add_compile_options(${CXX_FLAGS})


find_package(ament_cmake REQUIRED)
find_package(rclcpp_components)
find_package(hippo_msgs REQUIRED)
find_package(hippo_common REQUIRED)
find_package(Protobuf REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)

set(msgs_protos
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/hippo_gz_plugins/range_measurement.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/hippo_gz_plugins/range_measurement_array.proto
)
gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "gz.hippo_gz_plugins"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  MSGS_PROTOS ${msgs_protos}
  DEPENDENCIES gz-msgs
)
include_directories(${CMAKE_BINARY_DIR})

# find_package(ignition-msgs8 REQUIRED)
# set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

# find_package(Protobuf REQUIRED)
# set(Protobuf_IMPORT_DIRS ${ignition-msgs8_INCLUDE_DIRS})
# add_subdirectory(proto/msgs)
# set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)
# ament_export_include_directories(${CMAKE_BINARY_DIR}/proto/msgs)


function(add_plugin plugin_name)
  add_library(${plugin_name} SHARED
  src/${plugin_name}/${plugin_name}.cpp
  src/${plugin_name}/${plugin_name}_private.cpp
  )
  target_include_directories(${plugin_name} PRIVATE include)
  target_link_libraries(${plugin_name}
    gz-plugin::core
    gz-sim::core
    gz-common::core
    gz-msgs::core
    gz-transport::core
    hippo_gz_plugins-msgs
  )

  install(TARGETS ${plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
endfunction()

################################################################################
## range_sensor_bridge composable node
################################################################################
set(component_name range_sensor_bridge)
add_library(${component_name}
  SHARED
  src/range_sensor_bridge/range_sensor_bridge.cpp
)
target_include_directories(${component_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${component_name}
  hippo_gz_plugins-msgs
  gz-transport::core
)

ament_target_dependencies(${component_name}
  rclcpp
  rclcpp_components
  hippo_common
  hippo_msgs
)
rclcpp_components_register_node(${component_name} PLUGIN "hippo_gz_plugins::range_sensor_bridge::RangeSensor" EXECUTABLE ${component_name}_node)
install(TARGETS ${component_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_plugin(barometer)
add_plugin(buoyancy)
add_plugin(odometry)
add_plugin(thruster)
add_plugin(range_sensor)
# add_subdirectory(src/barometer)
# add_subdirectory(src/buoyancy)
add_subdirectory(src/hydrodynamics)
# add_subdirectory(src/odometry)
add_subdirectory(src/pose)
add_subdirectory(src/kinematic_control)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
