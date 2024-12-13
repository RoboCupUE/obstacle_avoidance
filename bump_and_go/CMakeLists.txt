cmake_minimum_required(VERSION 3.8)
project(bump_and_go)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

#######################
# robot_mover library #
#######################

add_library(robot_mover SHARED
  src/${PROJECT_NAME}/robot_mover.cpp
)

ament_target_dependencies(robot_mover
  rclcpp
  std_msgs
  geometry_msgs
)

##############################
# collision_detector library #
##############################

add_library(collision_detector SHARED
  src/${PROJECT_NAME}/collision_detector.cpp
)

ament_target_dependencies(collision_detector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  laser_geometry
  tf2
  tf2_ros
)

####################
# robot_mover_node #
####################

add_executable(robot_mover_node src/robot_mover_node.cpp)
ament_target_dependencies(robot_mover_node
  rclcpp
)
target_link_libraries(robot_mover_node
  robot_mover
)
#target_include_directories(robot_mover_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

###########################
# collision_detector_node #
###########################

add_executable(collision_detector_node src/collision_detector_node.cpp)
ament_target_dependencies(collision_detector_node
  rclcpp
)
target_link_libraries(collision_detector_node
collision_detector
)
#target_include_directories(collision_detector_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  robot_mover
  collision_detector
  robot_mover_node
  collision_detector_node
  EXPORT export_robot_mover
  EXPORT export_collision_detector
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
