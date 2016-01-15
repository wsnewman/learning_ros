# example_joint_controller
This code illustrates how ROS can interact with Gazebo via Gazebo services.
The example code implements joint servo control by accessing joint state via the service
/gazebo/get_joint_properties and exerting joint torques via the service
/gazebo/apply_joint_effort

## Example usage
Start Gazebo with:
`roslaunch gazebo_ros empty_world.launch`

From the minimal_robot_description package, spawn the URDF into Gazebo with:
`rosrun gazebo_ros spawn_model -urdf -file minimal_robot_description_wo_collision.urdf -model one_DOF`

Start the minimal joint controller running with:
`rosrun minimal_joint_controller minimal_joint_controller`

Change the setpoint angle command to the controller, e.g. to 1.0 rad with:
`rostopic pub pos_cmd std_msgs/Float64 1.0`


    
