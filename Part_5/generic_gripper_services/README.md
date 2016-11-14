# generic_gripper_services
This package is intended to hold gripper interface services.  A service should present a generic
interface, as defined by the service-message herein.  However, each type of gripper requires its own, specific
driver.  The example rethink_gripper_service.cpp applies to a parallel-jaw gripper on Baxter's right hand.
This driver presents a service called generic_gripper_svc, which expects messages using the 
genericGripperInterface message.  Action codes are defined within this message.  The return should be 
success=true.  

A similar service could be constructed for other grippers, e.g. a vacuum gripper.  

Optional parameters are available in the gripper service message that can be used to evaluate successful grasp.

## Example usage
Start up baxter:
`roslaunch baxter_gazebo baxter_world.launch`
Enable the robot (this is required for grippers as well as joint motors):
`rosrun baxter_tools enable_robot.py -e`
Start the gripper service for this type of gripper (on right hand):
`rosrun generic_gripper_services rethink_rt_gripper_service`
Run an example client program:
`rosrun generic_gripper_services example_generic_gripper_client`
This should open and close the right gripper.

See, also, use in object_grabber, object_grabber3.cpp

    
