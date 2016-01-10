cmake_minimum_required(VERSION 2.8.3)
project(example_ros_msg)

find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(example_ros_message_publisher src/example_ros_message_publisher.cpp)
cs_add_executable(vector_publisher src/vector_publisher.cpp)
cs_add_executable(vector_subscriber src/vector_subscriber.cpp)
cs_install()
cs_export()
