# simple_baxter_gripper_interface
The node test_baxter_gripper shows how to publish gripper commands to Baxter's right gripper.  
open/close values range from 100 to 0.

The library simple_baxter_gripper_interface defines a class, BaxterGripper, that contains useful
functions, including simplified open/close functions and filtering of gripper position values.  Use
of this library and its functions is illustrated with the node baxter_gripper_lib_test_main.cpp

## Example usage
bring up a Baxter simulator.  
`roslaunch baxter_gazebo baxter_world.launch`
Make sure robot is enabled:
`rosrun baxter_tools enable_robot.py -e`
Start the gripper test program:
`rosrun simple_baxter_gripper_interface test_baxter_gripper`
This will open/close the gripper with 1-second pauses.
OR, instead of test_baxter_gripper, use the library and test main:
`rosrun simple_baxter_gripper_interface baxter_gripper_lib_test_main`
This node illustrates commanding left and right grippers, as well as monitoring gripper position.  This
test node can be emulated to incorporate the BaxterGripper interface in other applications. 


    
