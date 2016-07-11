# arm7dof_traj_as
This package contains a trajectory action server to interface to the arm7dof robot.
The action server, arm7dof_traj_as, uses the library (defined in this package) arm7dof_trajectory_streamer, which
contains useful utilities for manipulating trajectory messages. 

The action server arm7dof_traj_as accepts trajectory goal messages, using the action
message defined in this package.  An interpolator within this action server interpolates linearly between successive
joint-space points.

An example action client, arm7dof_traj_action_client_prompter, uses the arm7dof_trajectory_streamer library
and sends trajectory goals to the arm action server.  This client prompts the user for a joint number and joint angle,
stuffs a trajectory message, and sends this goal to the trajectory action server for interpolation and execution.

## Example usage
Start the robot or the simulator, e.g.:
`roslaunch gazebo_ros empty_world.launch`
`roslaunch arm7dof_model arm7dof_w_pos_controller.launch`

Start the trajectory-interpolation action server:
`rosrun arm7dof_traj_as arm7dof_traj_as`

This action server should be run for all Arm7dof code examples provided here.  For an example, run the prompter client,
which prompts the user for values and commands the arm to move:
`rosrun arm7dof_traj_as arm7dof_traj_action_client_prompter`