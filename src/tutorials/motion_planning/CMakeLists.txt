cmake_minimum_required(VERSION 3.22)
project(lite6_moveit_demos)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/motion_planning_python_api_tutorial.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
