# localization_w_gps

Package to illustrate how to combine odometry, IMU and GPS data into merged
estimate of localization (without benefit of a LIDAR and map)

## Example usage
start up Gazebo:
`roslaunch gazebo_ros empty_world.launch`
start up a mobot with an IMU:
`roslaunch mobot_urdf mobot_w_imu.launch` 
  see topic: /imu_data (use z-component of angular velocity)

Start up noisy GPS emulation:
`rosrun mobot_gazebo_state mobot_gazebo_state2`
  see topic: /gazebo_mobot_noisy_pose
     use components position/x and position/y
     also publishes orientation as a yaw angle, useful for plotting
     
start up an imperfect odometry publisher: 
`rosrun mobot_drifty_odom mobot_drifty_odom` 
  see topic: /drifty_odom
   use components pose/position/x, position/y OR
    twist/linear/x   twist/angular/z (but IMU should be better than odom for z-rotation)
    
start up the localizer:
`rosrun localization_w_gps localization_w_gps`
result is published on topic /mobot_localization as a geometry_msgs/PoseStamped

Try moving the robot with:
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
and observe convergence of pose estimates with rqt_plot

## Running tests/demos
use rqt_plot to view  /gazebo_mobot_pose/position/x and /y
compare to mobot_localization/pose/position/x and /y
also plot /true_yaw and /yaw_estimate
/gazebo_mobot_noisy_pose/position/x and /y show the noisy gps values used

can change values of noise in gps, or add noise/offset to IMU omega_z;
can change feedback gains in localization_w_gps:
K_YAW should be between 0 and 1
K_GPS should be between 0 and 1
L_MOVE is length of displacement between gps-based yaw updates
   
