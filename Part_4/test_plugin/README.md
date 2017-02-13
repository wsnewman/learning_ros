This package illustates how to create a plug-in of nav_core that substitutes an alternative
base_local_planner.  The simple demo merely commands a 5-second motion with fixed speed/spin
each time a new global plan is received (after the 5-second motion is completed).

This simple test plugin does not use localization, does not perform steering, and in fact
ignores the global plan.  However, it could be upgraded to perform these functions.

A second version illustrates use of odom and tf (w/ amcl).  

See also README of package mobot_with_plugin:

Start this demo by first bringing up gazebo with starting pen and mobot:

(optirun) `roslaunch mobot_urdf mobot_in_pen.launch`

Then run the launch file contained in this package:

`roslaunch mobot_with_plugin mobot_startup_navstack.launch`

to display subgoals, run node:
`rosrun example_rviz_marker triad_display`

