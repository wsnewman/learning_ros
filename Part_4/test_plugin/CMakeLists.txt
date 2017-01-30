cmake_minimum_required(VERSION 2.8.3)
project(test_plugin)

find_package(catkin_simple REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_simple()

#Library
#Just make the one library, the plugin manager will handle the rest.
add_library(test_planner src/test_planner.cpp)
target_link_libraries(test_planner xform_utils  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

cs_install()
cs_export()
