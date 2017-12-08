cmake_minimum_required(VERSION 2.8.3)
project(example_rviz_marker)

find_package(catkin_simple REQUIRED)

catkin_simple()
#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Executables
cs_add_executable(example_rviz_marker src/example_rviz_marker.cpp)
cs_add_executable(example_rviz_line_markers src/example_rviz_line_markers.cpp)
cs_add_executable(triad_display src/triad_display.cpp)
cs_add_executable(lines_display src/lines_display.cpp)
cs_add_executable(triad_display_test_node src/triad_display_test_node.cpp)
cs_add_executable(linesegs_display_test_node src/linesegs_display_test_node.cpp)
cs_install()
cs_export()

