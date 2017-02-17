This package simply contains a launch file.  This launch file includes the line:
`<param name="base_local_planner" value="test_plugin/TestPlanner"/>`
which invokes use of a plug-in for the base_local_planner.
This plug-in resides in package `test_plugin`

Start this demo by first bringing up gazebo with starting pen and mobot:

(optirun) `roslaunch mobot_urdf mobot_in_pen.launch`

Then run the launch file contained in this package:

`roslaunch mobot_with_plugin mobot_startup_navstack.launch`

In the rviz display that is launched, use the "2D Nav Goal" tool to specify a goal pose.
Specifying a goal will cause a path to get computed and published (and displayed in rviz), and
it will initiate motion from the base_local_planner plug-in.  

In the test_plugin, the robot uses amcl and odom to reference its state relative to the global plan, 
and it performs steering feedback to follow the plan.  If an obstacle is perceived ahead, the
robot slows to a halt.










