cmake_minimum_required(VERSION 3.20)
project(wam_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wam_msgs REQUIRED)
#find_package(wam_srvs REQUIRED)
find_package(bhand_msgs REQUIRED)
#find_package(bhand_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})


##add executables
add_executable(wam_node
  src/wam_node.cpp
)
ament_target_dependencies(wam_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "trajectory_msgs"
  "Barrett"
  "std_srvs"
  "wam_msgs"
#  "wam_srvs"
  "bhand_msgs"
#  "bhand_srvs"
  "tf2"
  "tf2_ros"
)

target_link_libraries(wam_node ${BARRETT_LIBRARIES})

#install
install(TARGETS
  wam_node
DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
) 

###Define launch file
#install(DIRECTORY
 # launch
 # DESTINATION share/${PROJECT_NAME}
#) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

