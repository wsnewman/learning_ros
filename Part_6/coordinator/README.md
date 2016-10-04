# coordinator

Top-level node that is a client of: object_grabber, and object_finder.
Waits for an incoming goal, then starts entire process of: 
recognition of object, grasp of object, and dropoff of object.

The client specifies the object of interest, optionally with a specified pose 
(alternatively, requesting Kinect-based perception of the object).  The client
specifies the object's drop-off pose.  Grasp transforms are associated with
specified object ID's (via an object-properties library).

At present, this does not incorporate navigation.

start up an empty world:
`roslaunch gazebo_ros empty_world.launch`
 
 spawn Baxter on pedestal in front of a table and a block:
 `roslaunch baxter_variations baxter_on_pedestal_w_kinect.launch`

launch a bunch of nodes, including trajectory streamers, cartesian planner, rviz, baxter-playfile, triad_display (for object-frame visualization), object-grabber, object-finder coordinator, and block-state resetter:
`roslaunch coordinator baxter_object_grabber_nodes.launch`
    
Start up an example client node:
`rosrun coordinator coordinator_action_client3`

The robot will look for a block on the table and will plan and execute motions to
pick up the block and drop it off at a fixed dropoff location.  After each trial,
the block position will be reset randomly on the tabletop within reach of the robot.

The operation continues to repeat, and performance data is saved to the file "failures"


