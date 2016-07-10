# arm7dof_NAC_controller
Natural Admittance Controller for 7-DOF arm example.
Responds to force sensor; e.g., drop a weight and observe dynamics.

In this example, the (x,y,z) endpoint attractor is hard coded to (0,0,1.5),
and the wrist orientation is commanded to be constant.


## Example usage
Start up Gazebo:
`roslaunch gazebo_ros empty_world.launch`
Bring in the robot model with joint-velocity controllers:
`roslaunch arm7dof_model arm7dof_w_vel_controller.launch`
Initialize the pose by running:
`rosrun nested_loop_control inner_vel_loop`
then kill this node.

Start up the NAC controller:
`rosrun arm7dof_NAC_controller arm7dof_NAC_controller`

Reduce gravity to, e.g. -0.5m/sec^2, then drop a weight with:
`roslaunch exmpl_models add_cylinder_weight.launch`

