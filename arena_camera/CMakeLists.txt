cmake_minimum_required(VERSION 3.3)
project(arena_camera)

set(CMAKE_CXX_STANDARD 14)

#
# ARENA SDK
#
# ADDS :
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
  imaging_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()



#
# CATKIN
#

# set(CATKIN_COMPONENTS
#      actionlib
#      camera_control_msgs # arena_camera depends on this upper custom msg package
#      dynamic_reconfigure
#      camera_info_manager
#      cv_bridge
#      diagnostic_updater
#      image_geometry
#      image_transport
#      roscpp
#      roslint
#      roslaunch
#      sensor_msgs
#      nodelet
#      imaging_msgs
# )

# find_package(
#     catkin
#         REQUIRED
#     COMPONENTS
#         ${CATKIN_COMPONENTS}
# )

# catkin_python_setup()

#
# Generate dynamic reconfigure files
#
# generate_dynamic_reconfigure_options(
#   cfg/ArenaCamera.cfg
# )

# # http://wiki.ros.org/catkin/CMakeLists.txt#catkin_package.28.29
# catkin_package(
#     INCLUDE_DIRS
#         include
#     LIBRARIES
#         ${PROJECT_NAME}
#     CATKIN_DEPENDS
#         ${CATKIN_COMPONENTS} # package dep for build/run
#     # DEPENDS # sys dep for build/run
#     #    <>
#     #    <>
# )

#7
# ROS CHECKS
#

# set(ROSLINT_CPP_OPTS
#    "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
# )

# # check c/c++ static checking. More http://wiki.ros.org/roslint
# # no files mean all files
# roslint_cpp( # all .h .cpp files
# )

# # closest doc (http://wiki.ros.org/rosbuild/CMakeLists#rosbuild_add_roslaunch_check)
# # more https://answers.ros.org/question/200359/purpose-of-roslaunch_add_file_check/
# roslaunch_add_file_check(launch)


# -----------------------------------------------------------------------------
#
# 8
#
# -----------------------------------------------------------------------------


#  flags for all C++ targets
set(arena_camera_lib ${PROJECT_NAME})

add_library( ${arena_camera_lib}
    SHARED
    src/encoding_conversions.cpp
    src/arena_camera_parameter.cpp
    src/base_node.cpp
    src/polled_node.cpp
    src/streaming_node.cpp
)

ament_target_dependencies(${arena_camera_lib} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories( ${arena_camera_lib}
    PRIVATE ${arena_sdk_INCLUDE_DIRS}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries( ${arena_camera_lib}
    ${arena_sdk_LIBRARIES}
)

rclcpp_components_register_nodes(${arena_camera_lib} "arena_camera::ArenaCameraPolledNode")
rclcpp_components_register_nodes(${arena_camera_lib} "arena_camera::ArenaCameraStreamingNode")

## Build stand-alone executables
foreach(Executable IN ITEMS polled_node streaming_node)

  add_executable(${Executable} src/nodes/${Executable}_main.cpp)

  target_include_directories( ${Executable}
      PRIVATE ${arena_sdk_INCLUDE_DIRS}
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(${Executable} ${arena_camera_lib})

endforeach()


# Install
install(
  TARGETS ${arena_camera_lib}
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   include/
#   DESTINATION include
# )

install(DIRECTORY
  launch
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


# #
# # ARENA_CAMERA_NODES
# #

# set(streaming_arena_camera_node streaming_arena_camera)
# set(polled_arena_camera_node polled_arena_camera)

# add_executable( ${streaming_arena_camera_node}
#     src/nodes/streaming_node_main.cpp
# )

# add_executable( ${polled_arena_camera_node}
#     src/nodes/polled_node_main.cpp
# )

# foreach( camera_node ${streaming_arena_camera_node} ${polled_arena_camera_node} )

#     target_include_directories( ${camera_node}
#         PRIVATE ${catkin_INCLUDE_DIRS}
#     )

#     target_link_libraries( ${camera_node}
#         ${catkin_LIBRARIES}
#     )

# endforeach()


#
# WRITE_DEVICE_USER_ID_TO_CAMERA
#

# set(write_device_user_id_to_camera write_device_user_id_to_camera)

# # - declare an executable
# # - needed ".cpp"s
# add_executable(
#     ${write_device_user_id_to_camera} # run by rosrun
#         src/write_device_user_id_to_camera.cpp
# )

# # needed ".h"s
# target_include_directories(
#     ${write_device_user_id_to_camera}
#         PRIVATE ${arena_sdk_INCLUDE_DIRS}
#         PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# # needed ".so"s
# target_link_libraries(
#     ${write_device_user_id_to_camera}
#         ${arena_sdk_LIBRARIES}
# )


# target_link_directories NOT NEEDED as arena_sdk_LIBRARIES is
# define with absolute paths

# add_executable( dump_genicam_nodes
#     tools/dump_genicam_nodes.cpp
# )

# # needed ".h"s
# target_include_directories( dump_genicam_nodes
#     PRIVATE ${arena_sdk_INCLUDE_DIRS}
# )

# # needed ".so"s
# target_link_libraries( dump_genicam_nodes
#     ${arena_sdk_LIBRARIES}
# )


# -----------------------------------------------------------------------------
#
# 9
#
# -----------------------------------------------------------------------------

# N/A

# -----------------------------------------------------------------------------
#
# 10
#
# -----------------------------------------------------------------------------

# install(
#     DIRECTORY
#         launch/
#     DESTINATION
#         ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#     FILES_MATCHING PATTERN "*.launch"
# )

# install(
#     DIRECTORY
#         config/
#     DESTINATION
#         ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#     FILES_MATCHING PATTERN "*.yaml"
# )

# install(
#     PROGRAMS
#         scripts/file_sequencer.py
#         scripts/grab_and_save_image_action_server.py
#         scripts/result_bag_to_action.py
#         scripts/sequence_to_file.py
#         scripts/toggle_camera
#         scripts/triggered_image_topic.py
#     DESTINATION
#      ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(
#     TARGETS
#         ${streaming_arena_camera_node}
#         ${polled_arena_camera_node}
#         ${write_device_user_id_to_camera}
#         dump_genicam_nodes
#     LIBRARY DESTINATION
#         ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION
#         ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#     DIRECTORY
#         include/${PROJECT_NAME}/
#     DESTINATION
#         ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN
#         "*.h"
#     PATTERN "internal" EXCLUDE
# )

# ## Testing ##
# # All Jenkins-Tests are now in the arena_camera_tests-pkg
# #############

# ###############
# ## QtCreator ##
# ###############
# # entry for QtCreator to show all files
# file(GLOB children ${CMAKE_CURRENT_SOURCE_DIR}/*)
# foreach(child ${children})
#   if(IS_DIRECTORY ${child})
#     file(GLOB_RECURSE dir_files "${child}/*")
#     list(APPEND ${PROJECT_NAME}_extra_files ${dir_files})
#   endif()
# endforeach()
# #add_custom_target(dummy_${PROJECT_NAME} SOURCES ${${PROJECT_NAME}_extra_files})
