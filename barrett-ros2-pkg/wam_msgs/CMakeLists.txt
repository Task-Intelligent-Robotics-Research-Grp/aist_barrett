cmake_minimum_required(VERSION 3.20.0)
project(wam_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
  DEPENDENCIES
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

ament_auto_package()
