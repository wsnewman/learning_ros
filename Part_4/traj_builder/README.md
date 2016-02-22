# traj_builder
This library contains functions to build simple navigation trajectories.
The main function is: build_point_and_go_traj().  This function takes
arguments of a start pose, a goal pose, and a reference to a vector of
nav_msgs::Odom objects, which it will fill with a sequence of states that
correspond to a smooth and executable trajectory from start to goal.
This simple function will build a trajectory that does the following:
*find heading from start to goal coordinates
*compute a spin-in-place trajectory to point the robot towards the goal pose
*compute a forward-motion trajectory to move straight towards the goal pose
The trajectories are stored in a vector of Odometry objects, sampled every time
step dt_ (defaults to 20ms, but settable via fnc set_dt(double dt) ).
The Odom objects contain incremental states of x, y, orientation, translational
velocity and rotational velocity
For the spin and translation trajectories, these are either triangular-velocity
profiles, or trapezoidal-velocity profiles, determined by speed and accel params.
These parameters are set to default values by the constructor, but they can be 
overridden via corresponding "set" functions.

Note that for the spin-in-place behavior, the goal orientation is the direction
from start coords to goal coords.  Thus, the orientation in the goal pose is
ignored.  HOWEVER, if the start and goal poses have nearly identical (x,y) values,
(as determined by the parameter "path_move_tol_"), then the goal pose is 
interpreted to contain the desired goal heading.  Thus, if one wants to
travel to a pose and reach both the specified coordinates and orientation, 
simply repeat the last pose value as an additional subgoal, and this will
lead to a computed trajectory that concludes with reorienting to the desired
final heading.

see example program: traj_builder_example_main.cpp for illustration of how to 
use this library.

## Example usage
`roslaunch gazebo_ros empty_world.launch`
`roslaunch mobot_urdf mobot.launch`
`rosrun traj_builder traj_builder_example_main` 

    