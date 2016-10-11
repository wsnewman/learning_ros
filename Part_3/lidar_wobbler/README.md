# lidar_wobbler
Contains a minimal model for a moving LIDAR.  Can command motions of LIDAR rotation via joint1.
View result in rviz (with persistence on laserscan topic).

## Example usage
`(optirun) roslaunch gazebo_ros empty_world.launch`
Optirun is an option (if needed) to invoke use of GPU, since LIDAR plug-in uses GPU.
`roslaunch lidar_wobbler lidar_wobbler.launch`
Puts lidar-wobbler model on parameter server, spawns model into Gazebo, spawns a joint position controller,
and starts robot-state publisher and joint-state publisher.
`rosrun lidar_wobbler wobbler_sine_commander`
Commands sinusoidal position commands to the LIDAR wobbler.  Respond to prompts with amplitude and frequency
(e.g. amp= 1.57, freq = 0.1).
`rosrun rviz rviz`
Set the fixed frame to "world", add a LaserScan display with topic set to "/scan".  Set Decay Time parameter >0
(e.g. 10sec).

demo (crude) point transforms with:
`rosrun lidar_wobbler lidar_transformer`
(try adding objects withing view of scanner and observe 3-D point values)


