# odom_tf
Package to illustrate how to use amcl to correct for odometry drift.  
Amcl provides a tf message from odom frame to map frame.  Use this to
transform odom to map coordinates.  

## Example usage
Start up gazebo:
`(optirun) roslaunch gazebo_ros empty_world.launch`
launch nodes to start up the mobot in the starting pen, along with open-loop control nodes:
`roslaunch mobot_urdf mobot_startup_open_loop.launch`

load a map of the starting pen:
`roscd maps/starting_pen`
`rosrun map_server map_server starting_pen_map.yaml`

start the imperfect (drifty) odom node; there is also an odom topic published by gazebo, but this
one is unrealistically good, failing to illustrate realistic odom drift
`rosrun mobot_drifty_odom mobot_drifty_odom`
Run AMCL to localize the robot in the map.  AMCL updates are only about 1Hz, and thus
unsuitable for steering feedback.
`rosrun amcl amcl`
The odom_tf node illustrates how to combine AMCL information with imperfect odom information
to achieve smooth pose estimates, rapidly updated, with no cumulative drift.  
`rosrun odom_tf odom_tf_demo`
Start a desired-state publisher:
`rosrun mobot_pub_des_state mobot_pub_des_state`

## Running tests/demos
In rviz, give the robot an approximate starting pose.
Move the robot (under open-loop control), so AMCL will publish updated pose estimates.
`rosrun mobot_pub_des_state pub_des_state_path_client_3x3`  

In Rviz, display several frames to visualize how odom_tf is working.  Set the fixed frame to "map".
Display axes for "base_link", "amcl_base_link", "est_base" and "drifty_odom".
These will illustrate how amcl computes robot-pose updates infrequently, how the drifty_odom frame
moves around in the map, and how combining these to get "est_base" results in smooth robot-pose
estimates with no cumulative drift.

## Steering with merged AMCL/odometry
See the "lin_steering" package for illustration of using the OdomTf library for steering.
  
