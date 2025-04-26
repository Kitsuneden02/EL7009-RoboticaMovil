cmake_minimum_required(VERSION 3.8)

project(simple_manipulator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/forward_kinematics.py
  scripts/inverse_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
