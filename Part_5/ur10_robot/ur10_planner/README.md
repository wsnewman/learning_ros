# ur10_planner
Generic action server interface for UR10 robot.  Follows pattern of irb120_planner.
Defines robot-specific info in robot-specific header files, and overloads virtual 
functions for FK/IK using UR10 forward/inverse kinematic functions.

## Example usage
start up ur10 gazebo sim:
roslaunch ur_gazebo ur10.launch

start up generic transform publications:
roslaunch ur10_planner ur10_static_transforms.launch

start up the cartesian move action server:
rosrun ur10_planner ur10_cartesian_move_as 

start up an action client example; e.g., ac3 does raster motions
rosrun cartesian_motion_commander example_generic_cartesian_move_ac3


## Running tests/demos
    
