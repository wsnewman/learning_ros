cmake_minimum_required(VERSION 2.8.3)
project(example_rviz_marker)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(example_rviz_marker src/example_rviz_marker.cpp)

cs_install()
cs_export()

