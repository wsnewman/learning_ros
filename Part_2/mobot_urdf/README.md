# mobot_urdf
Contains mobot.xacro model of wheelchair-like robot.  Also has a launch file.

## Example usage
Start gazebo with an empty world:
`roslaunch gazebo_ros empty_world.launch`
then load the robot model into Gazebo via the parameter server:
`roslaunch mobot_urdf mobot.launch`
Drive the robot around, e.g. with:
`rostopic pub cmvel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.3}}'`

Launch file for this example does not include joint-state and robot-state publishers for use with rviz.

    
