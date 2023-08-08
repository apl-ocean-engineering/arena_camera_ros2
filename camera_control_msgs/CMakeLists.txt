cmake_minimum_required(VERSION 3.3)
project(camera_control_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# set(MSG_FILES)

# file(
#   GLOB rosMsgFiles
#   RELATIVE "${${PROJECT_NAME}_SOURCE_DIR}"
#   "${${PROJECT_NAME}_SOURCE_DIR}/msg/*.msg")

# foreach(rosMsg ${rosMsgFiles})
#   message(STATUS "Processing message file: ${rosMsg}")
#   set(MSG_FILES ${MSG_FILES} ${rosMsg})
# endforeach(rosMsg)

rosidl_generate_interfaces(${PROJECT_NAME}
                            srv/GetCamProperties.srv
                            srv/SetBinning.srv
                            srv/SetBool.srv
                            srv/SetBrightness.srv
                            srv/SetExposure.srv
                            srv/SetGain.srv
                            srv/SetGamma.srv
                            srv/SetROI.srv
                            srv/SetSleeping.srv
                        DEPENDENCIES
                            sensor_msgs )

ament_export_dependencies(rosidl_default_runtime)

ament_package()

# cmake_minimum_required(VERSION 2.8.3)
# project(camera_control_msgs)

# set(MSG_DEPS
#   actionlib_msgs
#   sensor_msgs
# )

# find_package(catkin REQUIRED COMPONENTS ${MSG_DEPS} message_generation)

# add_action_files(DIRECTORY action)
# add_service_files(DIRECTORY srv)

# generate_messages(DEPENDENCIES ${MSG_DEPS})
# catkin_package(CATKIN_DEPENDS ${MSG_DEPS} message_runtime)
