# simple_baxter_arm_controller

This example program sends two hard-coded, 7DOF joint-space commands to Baxter's right arm.
The poses are NOT interpolated, thus resulting in jerky motion

## Example usage
start up and enable the Baxter simulator (or robot), e.g.:

`roslaunch  baxter_gazebo baxter_world.launch`
Enable the robot:
`rosrun baxter_tools enable_robot.py -e`
Run the example arm controller with:
` rosrun simple_baxter_arm_controller simple_baxter_arm_controller`

Note: smoother motions can be achieved using a joint-space interpolator.  See, e.g., 

## Running tests/demos
    
