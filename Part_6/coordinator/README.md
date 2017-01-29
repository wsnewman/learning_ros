# coordinator

Top-level node that is a client of: object_grabber, and object_finder.
Waits for an incoming goal, then starts entire process of: 
recognition of object, grasp of object, and dropoff of object.

The client specifies the object of interest, optionally with a specified pose 
(alternatively, requesting Kinect-based perception of the object).  The client
specifies the object's drop-off pose.  Grasp transforms are associated with
specified object ID's (via an object-properties library).

start up an empty world:
`roslaunch gazebo_ros empty_world.launch`
 
 spawn Baxter on pedestal in front of a table and a block:
 `roslaunch baxter_variations baxter_on_pedestal_w_kinect.launch`

launch multiple nodes, including trajectory streamers, cartesian planner, rviz, baxter-playfile, triad_display (for object-frame visualization), object-grabber, object-finder coordinator, and block-state resetter:
`roslaunch coordinator coord_vision_manip.launch`
(to try object_grabber v2, run: roslaunch coordinator coord_vision_manip2.launch)
(also, v2 will respond to the client: rosrun coordinator example_cart_move_client,
which will induce a Cartesian gripper motion to a specified goal pose)

Command the robot to: find the table height, find the block on the table, compute an approach and grasp strategy,
execute the plan, and retract (holding the block) to the pre-pose position.
`rosrun coordinator acquire_block_client`

(to straddle the block, run: rosrun coordinator straddle_block_client)

The block can be placed back down again with the node:
`rosrun coordinator dropoff_block_client`

Instead of manually commanding pickup and dropoff commands, 
the entire process can be repeated continuously with:
`rosrun coordinator coordinator_action_client_tester`

With this node, the robot will look for a block on the table and will plan and execute motions to
pick up the block and drop it off at a fixed dropoff location.  After each trial,
the block position will be reset randomly on the tabletop within reach of the robot.
The operation continues to repeat, and performance data is saved to the file "failures"

Mobile manipulation:
(optirun) `roslaunch baxter_variations baxter_on_mobot.launch`

Start up the manipulator controls and nav-stack.  Wait for simulator to stabilize.  Then launch:
`roslaunch coordinator command_bundler.launch`

The above launch file includes the launchfile: baxter_variations/mobot_startup_navstack.launch`

The launch sequence may result in the blocks having fallen to the floor.  Reset the model
poses via Gazebo using Edit->reset model poses.

The following commands can be run manually, one at a time.  Alternatively, these
commands may be integrated into a single node.

As launched above, the robot will be facing a table with a block on the table.  Command
the robot to: get the table height, find the block on the table, compute a sequence of motion
commands to approach the block from above, grasp the block, depart, and move to the pre-pose position.
`rosrun coordinator acquire_block_client`
Next, manually command the robot to back up 1m:
`rosservice call open_loop_nav_service -- -1`
Command the robot to rotate CCW by 1 rad, causing it to face towards the starting-pen exit: 
`rosservice call open_loop_yaw_service 1`
Send a destination goal to move_base to approach a table outside the pen (still carrying the grasped block).
`rosrun example_move_base_client example_move_base_client`
Manually command the robot to approach closer to the table (a pose that would be forbidden by the
costmap):
`rosservice call open_loop_nav_service 0.7`
(should be at approx (-8.9, -0.15, 0)= (x,y,z) and (0,0,-90) = RPY base_link w/rt map)
Command the robot to stack the block by: perceiving the table surface, finding a block on the table,
computing a drop-off pose to stack the grasped block, computing arm motion commands to stack the block,
and executing this plan.
`rosrun coordinator stack_block_client`

These operations are combined in a single client program as well.  After launching coordinator/command_bundler.launch,
run:
`rosrun coordinator fetch_and_stack_client`
which picks up a block from table-1, navigates to table-2, and stacks the block on top of a block already on table-2.
