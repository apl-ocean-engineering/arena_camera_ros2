cmake_minimum_required(VERSION 3.3)
project(camera_control_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(sensor_msgs REQUIRED)

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
