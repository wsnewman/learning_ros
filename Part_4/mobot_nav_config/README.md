# mobot_nav_config
Config and launch files for running mobot model with LIDAR in map of starting pen.

## Example usage
(optirun) `roslaunch mobot_urdf mobot_in_pen.launch`
`roslaunch mobot_nav_config mobot_startup_navstack.launch`

send nav goals from Gazebo; also, can teleop with:
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Running tests/demos
    
