# cartesian_planner
This package includes separate, but very similar Cartesian planners for Baxter, for arm7dof and for the UR10 robot.
There are multiple cartesian-plan options, including:

*specify start and end poses w/rt base.  Only orientation of end pose will be considered; orientation of start pose is ignored;
planned motion keeps orientation constant while moving in a straight line from start position to end position.

*specify start as a q_vec, and goal as a Cartesian pose (w/rt base).  Orientation of goal pose will be obtained quickly,
then preserved through the linear move.

*specify start as a q_vec, and desired delta-p Cartesian motion while holding R fixed at initial orientation

## Example usage
The node "example_arm7dof_cart_path_planner_main.cpp" shows how to use the Cartesian planner, which relies 
on support from a corresponding fk_ik library and from the generic joint-space planner.

Running this node will produce an output file "arm7dof_poses.dat" comprised of joint-space poses that produce Cartesian motion with
the tool flange orientation constant.   The desired motion is specified hard-coded in the main program
to maintain orientation of the tool flange pointing up while translating at y-desired, z-desired from x-start to x-end.
These values can be edited to test alternative motions.

There are corresponding example mains for baxter and ur10.

This package also hosts action servers for cartesian moves for baxter right arm and UR10.  (arm7dof to be added).
These present a generic interface that can be driven by a generic action client.
It is necessary to run a launch file to publish transforms from specific, named frames on robots (e.g.
right_hand to generic_gripper_frame and torso to system_ref_frame, etc).  These launch files are in the launch
sub-directory.

## Running tests/demos
`rosrun  cartesian_planner example_arm7dof_cart_path_planner_main`

For Baxter robot:
`roslaunch baxter_gazebo baxter_world.launch`
wait for start-up, then enable the actuators:
`rosrun baxter_tools enable_robot.py -e`
Start a trajectory-interpolation action server:
`rosrun baxter_trajectory_streamer rt_arm_as`
start the generic static transforms:
`roslaunch cartesian_planner baxter_static_transforms.launch`
start the cartesian-motion action server for Baxter's right arm:
`rosrun  cartesian_planner baxter_rt_arm_cart_move_as` 
start a generic action client:
`rosrun cartesian_planner example_generic_cart_move_ac`   

or, for UR10, start up the UR10 Gazebo simulation (or real robot):
`roslaunch ur_gazebo ur10.launch`
start static transforms publishers:
`roslaunch cartesian_planner ur10_static_transforms.launch`
start up the cartesian planner action server:
`rosrun cartesian_planner ur10_cart_move_as`
run a generic cartesian-motion action client (same as above): 
`rosrun cartesian_planner example_generic_cart_move_ac`

For arm7dof:
`roslaunch gazebo_ros empty_world.launch` 
`roslaunch arm7dof_model arm7dof_w_pos_controller.launch`
`rosrun arm7dof_traj_as arm7dof_traj_as`
`roslaunch cartesian_planner arm7dof_static_transforms.launch`
`rosrun cartesian_planner arm7dof_cart_move_as` NOT WORKING YET
`rosrun cartesian_planner example_generic_cart_move_ac` 


