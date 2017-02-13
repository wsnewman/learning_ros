# example_ros_class

This code illustrates how to use classes to make ROS nodes.

The object constructor can do the initialization work, including setting up subscribers, publishers and services.

Can use member variables to pass data from subscribers to other member functions

## Example usage
Can try this function manually with terminal commands, e.g.:
`rosrun example_ros_class example_ros_class`  

Then peek/poke the various I/O options from a command line (in 3 separate terminals) with:

`rostopic echo example_class_output_topic`
`rostopic pub -r 4 example_class_input_topic std_msgs/Float32 2.0`
`rosservice call example_minimal_service`


    