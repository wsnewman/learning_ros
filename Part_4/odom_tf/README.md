# odom_tf
Package to illustrate how to use amcl to correct for odometry drift.  
Amcl provides a tf message from odom frame to map frame.  Use this to
transform odom to map coordinates.  

## Example usage
`(optirun) roslaunch gazebo_ros empty_world.launch`
`roslaunch mobot_urdf mobot_startup_open_loop.launch`
`rosrun map_server map_server newMap.yaml`



## Running tests/demos
    
