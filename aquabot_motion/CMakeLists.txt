cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(aquabot_motion)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
src/cmd.py
src/planner.py
src/follower.py
src/astar.py
src/bitangents.py
DESTINATION lib/${PROJECT_NAME})

# Resources
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
