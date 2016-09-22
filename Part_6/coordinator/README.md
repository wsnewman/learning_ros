# coordinator

Top-level node that is a client of: navigator, object_grabber, and object_finder.
Waits for an Alexa trigger, then starts entire process of: navigation to table,
recognition of object, grasp of object, return to home

THIS NEEDS UPDATING
## Example usage
Start up the simulator with:
`roslaunch baxter_launch beer_world.launch`
Enable the robot with:
`rosrun baxter_tools enable_robot.py -e`
Start up CWRU nodes with:
`roslaunch baxter_launch_files baxter_nodes.launch`

Then trigger the behavior with:
`rostopic pub Alexa_codes std_msgs/UInt32 100`
    
