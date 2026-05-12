cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(aquabot_motion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
#ament_auto_find_build_dependencies()


add_executable(motion_node src/motion_node.cpp)
ament_target_dependencies(motion_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(motion_node Eigen3::Eigen)

install(PROGRAMS
src/cmd.py
src/planner.py
src/follower.py
src/astar.py
src/bitangents.py
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  motion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
