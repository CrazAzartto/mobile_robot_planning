cmake_minimum_required(VERSION 3.8)
project(robot_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/goal_publisher.py
  scripts/dynamic_mover.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
