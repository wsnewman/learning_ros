# test_baxter_gripper
gripper_publisher shows how to publish gripper commands to Baxter's right gripper.  open/close values range from 100 to 0.

## Example usage
bring up a Baxter simulator.  Make sure robot is enabled:
`rosrun baxter_tools enable_robot.py -e`
Start the gripper test program:
`rosrun test_baxter_gripper gripper_publisher`
This will open/close the gripper with 1-second pauses.

    
