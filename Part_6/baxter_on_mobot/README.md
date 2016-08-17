# baxter_on_mobot
combined Baxter robot and mobile base with LIDAR and Kinect
Required editing Baxter model files to remove most of Baxter's sensor drivers (due to conflict with LIDAR).
Still references libbaxter_gazebo_ros_control.so and all CAD files in baxter_description, so this could
break if these are updated incompatibly.

## Example usage
Start up a non-empty world: contains starting pen, cafe table and beer can:
 `optirun roslaunch baxter_on_mobot play_pen_world.launch` 
 Load the combined baxter/mobot model into parameter server, spawn into Gazebo, load the controllers, and
 start up left and right arm trajectory interpolation action servers:
 `roslaunch baxter_on_mobot baxter_on_mobot_w_lidar.launch` 
 (the above uses $(find baxter_on_mobot)/baxter_on_mobot.xacro)
  ->  $(find baxter_on_mobot)/mobot_base.xacro
  
  ->  $(find baxter_on_mobot)/baxter_base.urdf.xacro
    -> $(find baxter_on_mobot)/baxter_drives.gazebo.xacro
    -> package://baxter_description/meshes/...
 Should be able to see arm poses corresponding to Gazebo and LIDAR points pinging the walls in rviz:
  `rosrun rviz rviz`  


 Wait for gravity compensation turned off, then enable Baxter's motors with:
   `rosrun baxter_tools enable_robot.py -e`
 Test moves: 
  `roscd baxter_playfile_nodes` and from this directory, generate a right-arm motion with:
  `rosrun baxter_playfile_nodes baxter_playback shake.jsp`  (or try shy.jsp, wave.jsp...; runs to completion)
  view Kinect pointCloud points on outstretched arm--overlay is (unrealistically) perfect
     
 Cause the base to spin: (move the table to avoid interference)
  `rostopic pub cmd_vel geometry_msgs/Twist '{ angular: { z : 0.3}}'`   
  
 Test the right gripper: (will open/close at 1-second intervals)
  `rosrun test_baxter_gripper gripper_publisher`

    
