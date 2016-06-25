# example_joint_controller
This code illustrates how ROS can interact with Gazebo via Gazebo services.
The example code implements joint servo control by accessing joint state via the service
/gazebo/get_joint_properties and exerting joint torques via the service
/gazebo/apply_joint_effort

Additionally, a simple version shows how to add a model to Gazebo and initialize its dynamic state.

## Example usage
Start Gazebo with:
`roslaunch gazebo_ros empty_world.launch`

In Gazebo, in the "world" tab, select "physics" and set the gravity z component to 0.  This will
remove gravity, emulating free-fall conditions.

Add a simple model (a rectangular prism) with the example launch file:
`roslaunch minimal_joint_controller add_rect_prism.launch`
A large, rectangular prism will appear, hovering above the ground plane (with gravity turned off--else it
will fall to the ground).

Under the Gazebo "world" tab, expand "Models", which will show "rect_prism" as the model just added.

Initialize the state of this model with:
`rosrun minimal_joint_controller example_gazebo_set_state`
This will cause the prism to translate with an x-velocity of 1cm/sec and a spin about its z axis of 1 rad/sec.

From the minimal_robot_description package, spawn the URDF into Gazebo with:
`rosrun gazebo_ros spawn_model -urdf -file minimal_robot_description_wo_collision.urdf -model one_DOF`

Start the minimal joint controller running with:
`rosrun minimal_joint_controller minimal_joint_controller`

Change the setpoint angle command to the controller, e.g. to 1.0 rad with:
`rostopic pub pos_cmd std_msgs/Float64 1.0`


    
