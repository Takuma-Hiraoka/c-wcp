cmake_minimum_required(VERSION 3.0.2)
project(c-wcp_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  c-wcp
  choreonoid_viewer
  choreonoid_bullet
  choreonoid_cddlib
  ik_constraint2_bullet
  roslib
  )

find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidCWCPSamplePlugin SHARED
  src/CnoidCWCPSamplePlugin.cpp
  src/world_common.cpp
  src/jaxon_common.cpp
  src/sample0.cpp
  )
target_link_libraries(CnoidCWCPSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidCWCPSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
