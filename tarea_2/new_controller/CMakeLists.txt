cmake_minimum_required(VERSION 3.8)

project(new_controller)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
