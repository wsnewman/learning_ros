# object_grabber
Example action server to accept high-level commands to acquire objects.
Objects are defined via object codes in the action message.
A goal contains an object code and a pose describing the frame of the object.
The action server should accept these goal specifications and perform an appropriate
sequence of actions to acquire the specified object at the specified location.

Start up the simulator with:
`roslaunch baxter_launch beer_world.launch`
Enable the robot with:
`rosrun baxter_tools enable_robot.py -e`
Start up CWRU nodes with:
`roslaunch baxter_launch_files baxter_nodes.launch`

## Using the object-grabber action server:
Start up the object-grabber action server with:
`rosrun object_grabber object_grabber_as` 

Start up a client (e.g., the example client in this package):
`rosrun object_grabber example_object_grabber_ac`    
