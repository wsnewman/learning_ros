# mobot_nl_steering
This package has a node that computes a non-linear steering algorithm for the simple mobot model.
It is intended for illustrative purposes.  By default, it assumes the desired path is the world-frame x axis.
The state of the robot is provided by the node mobot_gazebo_state (in a package of the same name).  This node
merely monitors the gazebo model states topic and republishes the mobot state for use by the example controller.
More generally, the robot state should be computed by a state estimator.  

## Example usage
Start gazebo:
`roslaunch gazebo_ros empty_world.launch` 
load the robot model:
`roslaunch mobot_urdf mobot.launch`
start the mobot state publisher:
`rosrun mobot_gazebo_state mobot_gazebo_state`
run the controller:
`rosrun mobot_nl_steering mobot_nl_steering`
The robot's initial conditions can be changed interactively in gazebo.  The robot should converge
on the x axis, heading in the positive x direction.  States can be monitored by the published
values on topics heading, heading_cmd and lateral_err.

    
