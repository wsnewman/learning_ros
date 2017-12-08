# irb120_ctl_interface
Interface to mimick ROS-industrial trajectory download interface.

## Example usage
Start up simulation of irb120 robot:
`roslaunch irb120_description irb120.launch`
Start up the control interface in this package, which subscribes to the topic "joint_path_command":
`rosrun irb120_ctl_interface irb120_ctl_interface`
This node is now ready to receive and execute trajectory messages published to "joint_path_command".


## Running tests/demos
A simple, test example (with hard-coded joint values and arrival times) can be run with:
`rosrun irb120_ctl_interface test_traj_sender_irb120`    
