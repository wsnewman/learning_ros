# example_tf_listener

This code illustrates use of a transform listener, using the mobot model to refer to tf frames.
Shows how to interpret and multiply transforms, and how to transform poses.

## Example usage
start gazebo:
`roslaunch gazebo_ros empty_world.launch`
launch the mobot:
`roslaunch mobot_urdf mobot_w_arm_and_jnt_pub.launch`
start a state publisher:
`rosrun robot_state_publisher robot_state_publisher`
start the transform listener:
`rosrun example_tf_listener example_tf_listener`



    
