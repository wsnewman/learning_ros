This package illustates how to create a plug-in of nav_core that substitutes an alternative
base_local_planner.  A minimal version is given in package example_nav_plugin.  The version
provided here (which is incomplete) shows how to interact with more aspects of move_base.

The test planner here performs linear steering to keep a robot following the current global plan.
Functions within this example code show how one can access CostMaps constructed and maintained by
the navigation stack.  

The steering algorithm uses odometry and AMCL to compare the robot's pose to a desired pose from
the global path plan.  The robot will rotate and speed up or slow down to follow a stream of
desired states along the global path plan.  If an obstacle is detected along the path, the robot
will come to a halt before the obstacle.  

This planner achieves better steering of the mobot.  However, it is unfinished in that the local
planner does not offer perturbations to steer around unexpected obstacles.  Additional local planning,
such as a bug algorithm, should be incorporated.

See also README of package mobot_with_plugin:

Start this demo by first bringing up gazebo with starting pen and mobot:

(optirun) `roslaunch mobot_urdf mobot_in_pen.launch`

Then run the following launch file:

`roslaunch mobot_with_plugin mobot_startup_navstack.launch`

Subgoals along the global path will be displayed in rviz using the triad-display node from the
package example_rviz_marker.


