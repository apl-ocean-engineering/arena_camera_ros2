cmake_minimum_required(VERSION 2.8.3)
project(arena_imx490)

# ADDS :
# - ${_arena_sdk_ROOT}
# - ${arena_sdk_INCLUDE_DIRS} and ${arena_sdk_INCLUDES}
# - ${arena_sdk_LIBRARIES_DIRS} and ${arena_sdk_LIBS}
# - ${arena_sdk_FOUND}

find_package(arena_sdk QUIET)
if (NOT ${arena_sdk_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/../arena_camera//cmake/Findarena_sdk.cmake")
endif()

find_package(OpenCV REQUIRED)

#
# CATKIN
#

set(CATKIN_COMPONENTS
     actionlib
     camera_control_msgs # arena_camera depends on this custom msg package
     camera_info_manager
     cv_bridge
     diagnostic_updater
     image_geometry
     image_transport
     roscpp
     roslint
     roslaunch
     sensor_msgs
     arena_camera
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED
    COMPONENTS ${CATKIN_COMPONENTS}
)
# catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        ${CATKIN_COMPONENTS}
)

set(ROSLINT_CPP_OPTS
   "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
)

# check c/c++ static checking. More http://wiki.ros.org/roslint
# no files mean all files
roslint_cpp( # all .h .cpp files
)

# closest doc (http://wiki.ros.org/rosbuild/CMakeLists#rosbuild_add_roslaunch_check)
# more https://answers.ros.org/question/200359/purpose-of-roslaunch_add_file_check/
roslaunch_add_file_check(launch)

# -----------------------------------------------------------------------------

#  flags for all C++ targets
add_definitions("-std=gnu++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#
# ARENA_CAMERA_NODE
#

set( arena_imx490_lib_name ${PROJECT_NAME})

add_library( ${arena_imx490_lib_name}
    src/arena_imx490_nodelet.cpp
)

target_include_directories( ${arena_imx490_lib_name}
    PRIVATE ${catkin_INCLUDE_DIRS}
    PRIVATE ${arena_sdk_INCLUDE_DIRS}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries( ${arena_imx490_lib_name}
    ${catkin_LIBRARIES}
    ${arena_sdk_LIBRARIES}
)


add_dependencies( ${arena_imx490_lib_name}
    ${catkin_EXPORTED_TARGETS}
)


#
#
#

set(arena_imx490_node_name arena_imx490_node)

add_executable( ${arena_imx490_node_name}
    src/nodes/imx490_main.cpp
)

target_include_directories( ${arena_imx490_node_name}
    PRIVATE ${catkin_INCLUDE_DIRS}
)

target_link_libraries( ${arena_imx490_node_name}
    ${catkin_LIBRARIES}
)


#
# Sample IMX490 executables from LucidVision
# https://support.thinklucid.com/knowledgebase/tonemapping-images-tri054s-imx490/
#

add_executable( Cpp_Triton_OfflineProcessing
    tools/Cpp_Triton_OfflineProcessing.cpp
)

target_include_directories( Cpp_Triton_OfflineProcessing
    PRIVATE ${arena_sdk_INCLUDE_DIRS}
    PRIVATE ${arena_sdk_INCLUDE_DIRS}/../Save/
    PRIVATE ${OpenCV_INCLUDE_DIRS}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tools/
)

target_link_libraries( Cpp_Triton_OfflineProcessing
    ${arena_sdk_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable( Cpp_Triton_Recorder
    tools/Cpp_Triton_OfflineProcessing.cpp
)

target_include_directories( Cpp_Triton_Recorder
    PRIVATE ${arena_sdk_INCLUDE_DIRS}
    PRIVATE ${arena_sdk_INCLUDE_DIRS}/../Save/
    PRIVATE ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries( Cpp_Triton_Recorder
    ${arena_sdk_LIBRARIES}
    ${OpenCV_LIBS}
)


# -----------------------------------------------------------------------------

install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(
    DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

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

install(
    TARGETS
        ${arena_imx490_node_name}
        Cpp_Triton_Recorder
        Cpp_Triton_OfflineProcessing
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN  "*.h"
    PATTERN "internal" EXCLUDE
)

## Testing ##
# All Jenkins-Tests are now in the arena_camera_tests-pkg
#############
