# baxter_launch
Start up the baxter simulator and support nodes with:
`roslaunch baxter_launch beer_world.launch`

Start up action servers and tf publishers with:
`roslaunch baxter_launch_files baxter_nodes.launch`

This starts the right and left arm trajectory interpolation action servers, as well as the
cartesian-move action server.  It also starts publishing kinect and gripper transforms.

Need to enable the arms:
`rosrun baxter_tools enable_robot.py -e`

Can see the effects of these nodes by running the example client:
`rosrun cartesian_planner baxter_cart_move_action_client`

    
