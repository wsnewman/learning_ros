# baxter_trajectory_streamer
This package contains a library of useful utilities, baxter_trajectory_streamer.cpp, for controlling Baxter arms.
Also, two action servers: left_arm_as and rt_arm_as, which accept trajectory goal messages, using the action
message defined in this package.  The interpolators accept trajectories and interpolate linearly between successive
joint-space points.

An example action client, pre_pose (from traj_action_client_pre_pose.cpp), uses the baxter_trajectory_streamer library
and sends trajectory goals to the left and right arm action servers.  The example trajectories start from the
current arm poses and go to hard-coded goal poses.

## Example usage
Start the robot or the simulator, e.g.:
`roslaunch baxter_gazebo baxter_world.launch`
OR:
`roslaunch baxter_launch baxter_world.launch`
(to add cafe table and beer can)
Enable the robot:
`rosrun baxter_tools enable_robot.py -e`
Start the trajectory-interpolation action servers:
`rosrun baxter_trajectory_streamer rt_arm_as`
`rosrun baxter_trajectory_streamer left_arm_as`
These action servers should be run for all Baxter code examples provided here.  For an example, run the pre-pose client,
which commands both arms to a hard-coded initial pose (mirrored left and right arms):
`rosrun baxter_trajectory_streamer pre_pose`
