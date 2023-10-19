cmake_minimum_required(VERSION 3.22)
project(lite6_moveit_demos)

find_package(ament_cmake REQUIRED)

add_subdirectory(scripts)
add_subdirectory(notebooks)

ament_package()
