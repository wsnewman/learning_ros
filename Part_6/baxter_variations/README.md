# baxter_on_mobot
combined Baxter robot and mobile base with LIDAR and Kinect
Required editing Baxter model files to remove most of Baxter's sensor drivers (due to conflict with LIDAR).
Still references libbaxter_gazebo_ros_control.so and all CAD files in baxter_description, so model here could
break if these are updated incompatibly.

Also created baxter_on_pedestal, which is the Baxter simulator on a rigid support,
with the addition of a Kinect sensor.  This works with the object_grabber and
object_finder action servers.  See "coordinator" package.

## Example usage
Start up gazebo, add starting pen, tables and blocks, spawn baxter-on-mobot:  
 (optirun) `roslaunch baxter_variations baxter_on_mobot.launch` 

Wait for Baxter's controllers to load and simu to stabilize, then run the following (which will include
enabling the motors).  

Launch multiple nodes, including 6 action servers, 2 services,  and rviz (including trajectory streamers, 
cartesian planner, rviz, baxter-playfile,  triad_display, object-grabber, object-finder and coordinator).  Same
as coordinator for baxter on pedestal:
`roslaunch coordinator coord_vision_manip.launch`

Diagnostics for vision and manipulation: 
Test the right gripper: (will open/close at 1-second intervals)
  `rosrun test_baxter_gripper gripper_publisher`

Test with this example client of the coordinator:
`rosrun coordinator coordinator_action_client_tester`

Start the nav-stack.  
Start up map server (with starting-pen map), amcl, and move_base with 4 nav config files.  Nearly identical
to nav-launch in Part-4, except do not start up rviz (again):

`roslaunch baxter_variations mobot_startup_navstack.launch`

 (the above uses $(find baxter_variations)/baxter_on_mobot.xacro)
  ->  $(find baxter_variations)/mobot_base.xacro
  
  ->  $(find baxter_variations)/baxter_base.urdf.xacro
    -> $(find baxter_variations)/baxter_drives.gazebo.xacro
    -> package://baxter_description/meshes/...

Mobility diagnostics:     
 Cause the base to spin: (move the table to avoid interference)
  `rostopic pub cmd_vel geometry_msgs/Twist '{ angular: { z : 0.3}}'`   
or use keyboard teleop:
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
or use rviz tool "2D Nav Goal" to set planner goals.

To fix head pan manually:
`rostopic pub /robot/head/command_head_pan baxter_core_msgs/HeadPanCommand   '{target: 0.0, enable_pan_request: 1}'`
(may have to run this twice)

Alternatively, run the node:
`rosrun baxter_head_pan baxter_head_pan_zero`




  


    
