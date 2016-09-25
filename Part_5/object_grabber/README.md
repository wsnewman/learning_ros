# object_grabber
Example action server to accept high-level commands to acquire objects.
Objects are defined via object codes in the action message.
A goal contains an object code and a pose describing the frame of the object.
The action server should accept these goal specifications and perform an appropriate
sequence of actions to acquire the specified object at the specified location.

Start up the simulator with:
`roslaunch baxter_gazebo baxter_world.launch`
Add a table and block with:
`roslaunch exmpl_models add_table_and_block.launch`
Wait for robot to finish starting up, then run:
`roslaunch baxter_launch_files baxter_object_grabber_nodes.launch`

## Using the object-grabber action server:
Start up a client (e.g., the example client in this package):
`rosrun object_grabber example_object_grabber_ac` 

This example client node sends (hard-coded) poses to grab the toy block, lift it,
rotate it, place it on the table, then retract the gripper to a pre-pose (fixed) pose.

This example client is a stand-in for a more general, perception-based grabber client. 
(see "coordinator" package)  
