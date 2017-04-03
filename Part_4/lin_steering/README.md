# lin_steering
Steering algorithm using odometry.  For mobot, "odom" is perfect.  Neede to relax this
assumption.

If start with good initial conditions, linear steering algorithm will do a good job.
Can compare feedback controller to open-loop controller.

## Example usage
Start up gazebo, load the mobot model, desired-state publisher, desired-state client,
and linear-steering algorithm.
`roslaunch gazebo_ros empty_world.launch`
`roslaunch mobot_urdf mobot.launch`
`rosrun mobot_pub_des_state mobot_pub_des_state`
`rosrun mobot_pub_des_state pub_des_state_path_client`
`rosrun lin_steering lin_steering_wrt_odom`

### Linear Steering with integrated AMCL and (drifty) odometry
Start up the mobot in the starting pen with:
`roslaunch mobot_urdf mobot_in_pen.launch`
Then launch multiple nodes, including amcl, linear steering with respect to amcl, 
drifty_odom, rviz, triad-display, and mobot_pub_des_state.  Start these with:
`roslaunch odom_tf mobot_w_odom_tf.launch`
Then send a desired path, in terms of a sequence of via points, with:
`rosrun mobot_pub_des_state starting_pen_pub_des_state_path_client`

This will send the robot out of the starting pen using 3 via points.  Fine-grained
desired state publication is done with mobot_pub_des_state, which interpolates
incremental desired states between via points, including velocity ramping.  

The drifty_odom topic introduces significant errors in odometry, but these errors
are corrected by updating absolute poses based on LIDAR and the current map, using
AMCL.  


    
