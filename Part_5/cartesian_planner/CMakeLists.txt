cmake_minimum_required(VERSION 2.8.3)
project(cartesian_planner)

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
cs_add_library(cartesian_planner src/cartesian_planner.cpp)   
cs_add_library(arm7dof_cartesian_planner src/arm7dof_cartesian_planner.cpp) 

cs_add_executable(example_cart_path_planner_main src/example_cart_path_planner_main.cpp)
cs_add_executable(example_arm7dof_cart_path_planner_main src/example_arm7dof_cart_path_planner_main.cpp)
cs_add_executable(example_arm7dof_cart_path_planner_main2 src/example_arm7dof_cart_path_planner_main2.cpp)
cs_add_executable(example_arm7dof_cart_path_planner_main3 src/example_arm7dof_cart_path_planner_main3.cpp)
cs_add_executable(plan_playfile src/plan_playfile.cpp)
cs_add_executable(baxter_cart_move_as src/baxter_cart_move_as.cpp)
cs_add_executable(baxter_cart_move_action_client src/example_baxter_cart_move_action_client.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(example src/example.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)
target_link_libraries(example_cart_path_planner_main cartesian_planner ${catkin_LIBRARIES})
target_link_libraries(example_arm7dof_cart_path_planner_main arm7dof_cartesian_planner ${catkin_LIBRARIES})
target_link_libraries(example_arm7dof_cart_path_planner_main2 arm7dof_cartesian_planner ${catkin_LIBRARIES})
target_link_libraries(example_arm7dof_cart_path_planner_main3 arm7dof_cartesian_planner ${catkin_LIBRARIES})
target_link_libraries(plan_playfile cartesian_planner ${catkin_LIBRARIES})
target_link_libraries(baxter_cart_move_as cartesian_planner ${catkin_LIBRARIES})
cs_install()
cs_export()
    
