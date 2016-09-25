# coordinator

Top-level node that is a client of: navigator, object_grabber, and object_finder.
Waits for an incoming goal, then starts entire process of: (navigation to table),
recognition of object, grasp of object, (return to home)

start up an empty world:
`(optirun) roslaunch gazebo_ros empty_world.launch`

add table and block
`roslaunch exmpl_models add_table_and_block.launch`

spawn Baxter on mobot:
`roslaunch baxter_on_mobot baxter_on_mobot_w_lidar.launch`

launch a bunch of nodes, including trajectory streamers, cartesian planner, rviz, baxter-playfile, 
triad_display (for object-frame visualization), object-grabber, object-finder and coordinator:
`roslaunch coordinator baxter_object_grabber_nodes.launch`

Start an example client of the coordinator (later, to send orders for kits)
`rosrun coordinator coordinator_action_client`

optional: to add random blocks:

`roslaunch exmpl_models add_blocks.launch`    
`rosrun example_gazebo_set_state set_block_state`
`rosservice call set_block_state 5`  (for block 5)

notes: frequent failures with gripper fingers hitting block.  May try smaller block.
Also, should allow gripper orientation w/ x-axis anti-parallel to block x-axis as well.
Further, allow drop-off w/ block x-axis anti-parallel as option to open up more solutions.
