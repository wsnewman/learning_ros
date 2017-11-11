cmake_minimum_required(VERSION 2.8.3)
project(baxter_fk_ik)

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
cs_add_library(baxter_fk_ik src/baxter_fk_ik.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(baxter_reachability_from_above src/baxter_reachability_from_above.cpp)
cs_add_executable(test_baxter_jacobian_main src/test_baxter_jacobian_main.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)
target_link_libraries(baxter_reachability_from_above baxter_fk_ik ${catkin_LIBRARIES})
target_link_libraries(test_baxter_jacobian_main baxter_fk_ik ${catkin_LIBRARIES})

cs_install()
cs_export()
    
