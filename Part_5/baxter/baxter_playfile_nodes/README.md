# baxter_playfile_nodes
Handy functions to record and playback Baxter joint states.

`rosrun baxter_playfile_nodes get_and_save_jntvals`
will sample the current joint angles of left and right arms.  Each time the user
enters "1", the current joint angles will be appended to the files "baxter_r_arm_angs.txt"
and "baxter_l_arm_angs.txt".

The node "baxter_recorder" extends this to recording and saving joint trajectories (with
the actual timing of manual arm motions).  This node
samples the right-arm and left-arm joint angles,
and saves them to disk in the files "baxter_r_arm_traj.jsp" (joint-space playfile)
and "baxter_l_arm_traj.jsp". (control-C when done recording).

Trajectories recorded in this fashion can be played back with the playfile reader,
"baxter_playback."

## Example usage
With Baxter (sim or real) running, cd to a trajectory appropriate to save recordings.
Be careful to rename "baxter_r_arm_traj.jsp" and "baxter_l_arm_traj.jsp" to avoid overwriting previous recordings.
When ready to record, start:
`rosrun baxter_playfile_nodes baxter_recorder`
enter "1" at the program prompt, then move the arms in the desired trajectory (path and speed).
When done with recording, ctl-C.  The results will be in "merry_r_arm_traj.jsp" and "merry_l_arm_traj.jsp".

To play back joint-space trajectory files, start up the robot and enable it.  Start up the trajectory
interpolation action servers:
Start the trajectory-interpolation action servers:
`rosrun baxter_trajectory_streamer rt_arm_as`
`rosrun baxter_trajectory_streamer left_arm_as`

In another terminal, cd to the directory containing the desired trajectory filename(s).
Run the playback node, with command-line arguments for the right-arm trajectory and left-arm trajectory.
(If only 1 filename is provided, it will be interpreted and executed as the right-arm file). E.g.:
`rosrun baxter_playfile_nodes baxter_playback baxter_r_arm_traj.jsp`
(to move just the right arm).  Or,
`rosrun baxter_playfile_nodes baxter_playback baxter_r_arm_traj.jsp baxter_l_arm_traj.jsp`
to move both arms.



    
