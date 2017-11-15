# baxter_jnt_traj_ctlr_client
This package contains example nodes that illustrate use of the Baxter action server for trajectory control.
Baxter's trajectory controller can be started with:
`rosrun baxter_interface joint_trajectory_action_server.py --mode position`
One can then run:
`rosrun baxter_jnt_traj_ctlr_client baxter_jnt_traj_ctlr_client_pre_pose` to send the right arm to a hard-coded pose, or
`rosrun baxter_jnt_traj_ctlr_client baxter_jnt_traj_ctlr_client_home` to send the right arm to all joint-angles= 0.

These example client nodes show how to connect action clients to Baxter's action servers for trajectory
interpolation and execution.

If desired, pre-position arms (before starting joint_trajectory_action_server) with:
`rosrun baxter_tools tuck_arms.py -u`

Alternatively, demonstrate use of the Jacobian to execute approximate Cartesian motions with:
`rosrun baxter_jnt_traj_ctlr_client baxter_jnt_traj_ctlr_client_Jacobian_move`

Can observe result as follows.  Before running the Jacobian move, check gripper pose with: 
`rosrun tf tf_echo torso right_gripper`
Then, after performing move,  run tf_echo again and compare the results. E.g., start from pre-pose position:
`rosrun baxter_jnt_traj_ctlr_client baxter_jnt_traj_ctlr_client_pre_pose`

Then move in x direction by 200mm (see lines 46 and 48 of baxter_jnt_traj_ctl_client_Jacobian_move.cpp)
Observe 200mm motion in x dir, with only small translation y,z and little reorientation.

           







   
    
