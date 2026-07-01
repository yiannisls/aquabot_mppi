cmake_minimum_required(VERSION 3.18)

project(aquabot_motion LANGUAGES C CXX)

enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES native)

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
find_package(visualization_msgs REQUIRED)
find_package(OpenMP REQUIRED)
#ament_auto_find_build_dependencies()


add_executable(motion_node 
  src/motion_node.cpp
  src/mppi_kernels.cu
)
# add_executable(motion_node_rey src/motion_node_rey.cpp)
target_compile_features(motion_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(motion_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
target_link_libraries(motion_node Eigen3::Eigen)
target_link_libraries(motion_node OpenMP::OpenMP_CXX)

install(PROGRAMS
src/cmd.py
src/planner.py
src/follower.py
src/astar.py
src/bitangents.py
src/mission_turbines.py
src/qr_reader.py
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  motion_node
  # motion_node_rey
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
