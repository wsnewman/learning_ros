# example_ROS_service
This simple example shows how to write a ROS service node.  See the accompanying document, "Introduction to ROS services"

## Running tests/demos
 run this as: 
`rosrun example_ros_service example_ros_service`

in another window, tickle it manually with (e.g.): 
 `rosservice call lookup_by_name 'Ted'`   

Alternatively, run the corresponding client node with:

`rosrun example_ros_service example_ros_client`
