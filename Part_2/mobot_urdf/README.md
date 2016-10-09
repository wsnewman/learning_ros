# mobot_urdf
Contains mobot.xacro model of wheelchair-like robot.  Also has a launch file.

## Example usage
Start gazebo with an empty world:
`roslaunch gazebo_ros empty_world.launch`
then load the robot model into Gazebo via the parameter server:
`roslaunch mobot_urdf mobot.launch`
Drive the robot around, e.g. with:
`rostopic pub cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.3}}'`

Alternatively, launch Gazebo, spawn the starting-pen model, load and spawn the robot, and start a robot-state
publisher with:
`roslaunch mobot_urdf mobot_in_pen.launch`
Then drive the robot around under teleoperation with:
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

    
