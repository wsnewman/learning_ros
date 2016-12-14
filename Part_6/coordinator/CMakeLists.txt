cmake_minimum_required(VERSION 2.8.3)
project(coordinator)

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
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(command_bundler src/command_bundler.cpp)
cs_add_executable(acquire_block_client src/acquire_block_client.cpp)
cs_add_executable(straddle_block_client src/straddle_block_client.cpp)
cs_add_executable(example_cart_move_client src/example_cart_move_client.cpp)
cs_add_executable(dropoff_block_client src/dropoff_block_client.cpp)
cs_add_executable(coordinator_action_client_tester src/coordinator_action_client_tester.cpp)
cs_add_executable(open_loop_nav_service src/open_loop_nav_service.cpp)
cs_add_executable(open_loop_yaw_service src/open_loop_yaw_service.cpp)
cs_add_executable(fetch_and_stack_client src/fetch_and_stack_client.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)

cs_install()
cs_export()
    
