# object_grabber
Example action server to accept high-level commands to acquire objects.
Objects are defined via object codes in the action message.
A goal contains an object code and a stamped-pose describing the frame of the object.
The object-grabber action server accepts these goal specifications and performs appropriate
sequences of actions to acquire the specified object at the specified location and
drop off the object at a specified pose.

Start up the simulator with:
`roslaunch baxter_gazebo baxter_world.launch`
Add a table and block with:
`roslaunch exmpl_models add_table_and_block.launch`
Wait for robot to finish starting up, then run:
`roslaunch baxter_launch_files baxter_object_grabber_nodes.launch`

Try grabbing the block by running:
`rosrun object_grabber example_object_grabber_client` 

This example client node sends (hard-coded) poses to grab the toy block, lift it,
rotate it, place it on the table, then retract the gripper to a pre-pose (fixed) pose.
It then sends a drop-off command to place the block at a specified pose, release it,
and withdraw the gripper.

or, for UR10, start up the UR10 Gazebo simulation (or real robot).
Use this launch file to launch UR10 on a pedestal:
`roslaunch ur10_launch ur10_w_gripper.launch`

start up all nodes needed for object manipulation:
`roslaunch ur10_launch ur10_object_grabber_nodes.launch`

Try virtual block grasp (e.g., if had vacuum gripper on flange):
`rosrun object_grabber example_object_grabber_client` 


This example client is a stand-in for a more general, perception-based grabber client. 
(see "coordinator" package)  
