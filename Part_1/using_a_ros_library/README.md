# using_a_ros_library

This package illustrates how to link an executable to a custom ROS library.  The example is identical to the class defined in package example_ros_class and in package creating_a_ros_library.  The only difference with this package is that it only contains the
test main program, and it links with the library created in package creating_a_ros_library.

## Example usage
`rosrun using_a_ros_library ros_library_external_test_main`
starts the test executable.  

## Running tests/demos
As in the example ros class, behavior of the new test node can be examined using the commands:
`rostopic echo exampleMinimalPubTopic`
`rostopic pub -r 4 exampleMinimalSubTopic std_msgs/Float32 2.0`
`rosservice call exampleMinimalService 1`
    
