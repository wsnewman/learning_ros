cmake_minimum_required(VERSION 2.8.3)
project(pcl_utils)

find_package(catkin_simple REQUIRED)


#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

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
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(pcl_utils src/pcl_utils.cpp)  
#cs_add_library(xform_utils src/xform_utils.cpp) 
# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(example src/example.cpp)
#cs_add_executable(make_and_display_ellipse src/make_and_display_ellipse.cpp)
#cs_add_executable(pcl_simple_main src/pcl_simple_main.cpp)
#cs_add_executable(ransac_main src/ransac_main.cpp)
#cs_add_executable(example_process_pcd src/example_process_pcd.cpp)
cs_add_executable(display_ellipse src/display_ellipse.cpp src/make_clouds.cpp)
cs_add_executable(pcd_snapshot src/pcd_snapshot.cpp)
cs_add_executable(compute_selected_points_centroid src/compute_selected_points_centroid.cpp)
#cs_add_executable(demo_pcl1 src/demo_pcl1.cpp)
#cs_add_executable(demo_pcl2 src/demo_pcl2.cpp)

cs_add_executable(display_pcd_file src/display_pcd_file.cpp)
cs_add_executable(find_plane_pcd_file src/find_plane_pcd_file.cpp src/find_indices_of_plane_from_patch.cpp)
cs_add_executable(find_plane_pcd_file2 src/find_plane_pcd_file2.cpp src/find_indices_of_plane_from_patch2.cpp)
cs_add_executable(box_filter src/box_filter_main.cpp src/find_indices_of_plane_from_patch2.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)
#target_link_libraries(pcl_simple_main pcl_utils)
#target_link_libraries(ransac_main pcl_utils)
#target_link_libraries(pcd_snapshot pcl_utils)
#target_link_libraries(example_process_pcd pcl_utils)
target_link_libraries(find_plane_pcd_file pcl_utils)
target_link_libraries(find_plane_pcd_file2 pcl_utils)
target_link_libraries(box_filter pcl_utils)
cs_install()
cs_export()
    
