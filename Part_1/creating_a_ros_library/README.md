# creating_a_ros_library

This package illustrates how to create a library.  The example is identical to the class defined in package example_ros_class.
The header file, example_ros_class.h, is copied to the subdirectory /include/creating_a_ros_library/example_ros_class.h.  The
source code, example_ros_class.cpp, is copied to the subdirectory /src.  This code is edited to remove the main function
and to reference the header file as #include <creating_a_ros_library/example_ros_class.h>.

A separate test file, example_ros_class_test_main.cpp, contains the original main() function, and this file is linked to
the new library.

## Example usage
`rosrun creating_a_ros_library example_ros_class_test_main`
starts the test executable using the new library.  Note that the test main file is now quite brief, compared to the
source code in the package example_ros_class.  

See also the package using_a_ros_library for an example of linking to the library herein to an executable in a separate package.

## Running tests/demos
As in the example ros class, behavior of the new test node can be examined using the commands:
`rostopic echo example_class_output_topic`
`rostopic pub -r 4 example_class_input_topic std_msgs/Float32 2.0`
`rosservice call example_minimal_service`
    
