cmake_minimum_required(VERSION 2.8.3)
project(object_grabber)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library 
cs_add_library(object_grabber_lib src/object_grabber.cpp) 
cs_add_library(object_grabber_lib2 src/object_grabber2.cpp) 
# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(object_grabber_action_server src/object_grabber_as.cpp)
cs_add_executable(object_grabber_action_server2 src/object_grabber_as2.cpp)

cs_add_executable(example_object_grabber_action_client src/example_object_grabber_action_client.cpp)
cs_add_executable(block_grabber_action_client src/block_grabber_action_client.cpp)
cs_add_executable(set_baxter_gripper_param src/set_baxter_gripper_param.cpp)
cs_add_executable(set_sticky_fingers_gripper_param src/set_sticky_fingers_gripper_param.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package

target_link_libraries(object_grabber_action_server object_grabber_lib)
target_link_libraries(object_grabber_action_server2 object_grabber_lib2)
cs_install()
cs_export()
    
