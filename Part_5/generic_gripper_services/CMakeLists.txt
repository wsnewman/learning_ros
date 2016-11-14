cmake_minimum_required(VERSION 2.8.3)
project(generic_gripper_services)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(rethink_rt_gripper_service src/rethink_rt_gripper_service.cpp)
cs_add_executable(virtual_vacuum_gripper_service src/virtual_vacuum_gripper_service.cpp)
cs_add_executable(example_generic_gripper_client src/example_generic_gripper_client.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
#target_link_libraries(rethink_rt_gripper_service my_lib)

cs_install()
cs_export()
    
