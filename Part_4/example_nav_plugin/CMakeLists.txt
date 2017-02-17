cmake_minimum_required(VERSION 2.8.3)
project(example_nav_plugin)

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
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#Library
#Just make the one library, the plugin manager will handle the rest.
add_library(minimal_nav_plugin src/minimal_nav_plugin.cpp)
#target_link_libraries(minimal_nav_plugin xform_utils  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(example src/example.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

cs_install()
cs_export()
    
