cmake_minimum_required(VERSION 2.8.3)
project(object_manipulation_properties)

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
#cs_add_library(object_manipulation_properties src/object_manipulation_properties.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(object_manipulation_properties_test_main src/object_manipulation_properties_test_main.cpp)
cs_add_executable(object_manipulation_query_svc src/object_manipulation_query_svc.cpp)
cs_add_executable(example_object_manip_query_client src/example_object_manip_query_client.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
#target_link_libraries(object_manipulation_properties_test_main object_manipulation_properties)

cs_install()
cs_export()
    