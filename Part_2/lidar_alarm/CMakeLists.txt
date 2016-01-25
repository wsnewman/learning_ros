cmake_minimum_required(VERSION 2.8.3)
project(lidar_alarm)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(lidar_alarm src/lidar_alarm.cpp)
#cs_add_executable(lidar_alarm2 src/lidar_alarm2.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    
