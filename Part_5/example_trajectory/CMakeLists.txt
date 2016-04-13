cmake_minimum_required(VERSION 2.8.3)
project(example_trajectory)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(example_trajectory_action_server src/example_trajectory_action_server.cpp)
cs_add_executable(example_trajectory_action_client src/example_trajectory_action_client.cpp)

cs_install()
cs_export()
    
