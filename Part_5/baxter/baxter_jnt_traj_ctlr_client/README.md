# baxter_jnt_traj_ctlr_client
This package contains example nodes that illustrate use of the Baxter action server for trajectory control.
Baxter's trajectory controller can be started with:
`rosrun baxter_interface joint_trajectory_action_server.py --mode position`
One can then run:
`rosrun baxter_jnt_traj_ctlr_client baxter_jnt_traj_ctlr_client_pre_pose` to send the right arm to a hard-coded pose, or
`rosrun baxter_jnt_traj_ctlr_client baxter_jnt_traj_ctlr_client_home` to send the right arm to all joint-angles= 0.

These example client nodes show how to connect action clients to Baxter's action servers for trajectory
interpolation and execution.  
    