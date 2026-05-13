cmake_minimum_required(VERSION 3.5.0)
project(aist_barrett)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
#  Dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})

###########
#  Build  #
###########
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
    src/barrett_hand_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${BARRETT_LIBRARIES})

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
    PLUGIN     ${PROJECT_NAME}::BarrettHandController
    EXECUTABLE parrett_hand_controller_node
)

########################
#  Python executables  #
########################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/joint_state_to_array.py
    scripts/test_client.py
  DESTINATION
    lib/${PROJECT_NAME}
)

##############################
#  environment hooks for gz  #
##############################
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in"
)

ament_auto_package(INSTALL_TO_SHARE launch meshes urdf)
