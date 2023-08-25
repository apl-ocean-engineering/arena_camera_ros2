cmake_minimum_required(VERSION 3.3)
project(arena_camera)

set(CMAKE_CXX_STANDARD 14)

#
# ARENA SDK
#
# creates these variables:
# - ${_arena_sdk_ROOT}
# - ${arena_sdk_INCLUDE_DIRS} and ${arena_sdk_INCLUDES}
# - ${arena_sdk_LIBRARIES_DIRS} and ${arena_sdk_LIBS}
# - ${arena_sdk_FOUND}

find_package(arena_sdk QUIET)
if (NOT ${arena_sdk_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Findarena_sdk.cmake")
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  ament_cmake
  camera_control_msgs
  camera_info_manager
  image_transport
  imaging_msgs
  generate_parameter_library
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  arena_camera_parameters # cmake target name for the parameter library
  src/parameters.yaml     # path to input yaml file
)

set(arena_camera_lib ${PROJECT_NAME})

add_library( ${arena_camera_lib}
    src/encoding_conversions.cpp
    src/base_node.cpp
    src/polled_node.cpp
    src/streaming_node.cpp
)
ament_target_dependencies(${arena_camera_lib} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories( ${arena_camera_lib}
    PRIVATE ${arena_sdk_INCLUDE_DIRS}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries( ${arena_camera_lib}
    ${arena_sdk_LIBRARIES}
    arena_camera_parameters
)

#set_target_properties(${arena_camera_lib} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# rclcpp_components_register_nodes(${arena_camera_lib} "arena_camera::ArenaCameraPolledNode")
# rclcpp_components_register_nodes(${arena_camera_lib} "arena_camera::ArenaCameraStreamingNode")

## Build stand-alone executables
foreach(Executable IN ITEMS polled_node streaming_node)

  add_executable(${Executable} src/nodes/${Executable}_main.cpp)
  target_include_directories( ${Executable}
      PRIVATE ${arena_sdk_INCLUDE_DIRS}
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(${Executable} ${arena_camera_lib})
endforeach()

add_executable( dump_genicam_nodes
    tools/dump_genicam_nodes.cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Installation of the library has an unexpected dependency on
# "arena_camera_parameters".   Resolve if downstream packages
# ever want to use the arena_camera library.
install(
  TARGETS ${arena_camera_lib}
#  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
# ament_export_targets(export_${arena_camera_lib} HAS_LIBRARY_TARGET)
# ament_export_dependencies(arena_camera_parameters)

install(
  TARGETS dump_genicam_nodes
  DESTINATION bin
)

install(
  TARGETS polled_node streaming_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Run linters found in package.xml except those below
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_uncrustify_FOUND TRUE)
#   set(ament_cmake_pep257_FOUND TRUE)
#   set(ament_cmake_flake8_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
