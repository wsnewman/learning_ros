# baxter_playfile_nodes
Handy functions to record and playback Baxter joint states.

`rosrun baxter_playfile_nodes get_and_save_jntvals`
will sample the current joint angles of left and right arms.  Each time the user
enters "1", the current joint angles will be appended to the files "baxter_r_arm_angs.txt"
and "baxter_l_arm_angs.txt".


Can record joint-space trajectories with:
`rosrun baxter_playfile_nodes baxter_recorder`

This node samples the right-arm and left-arm joint angles,
and saves them to disk in the files "baxter_r_arm_traj.jsp" (joint-space playfile)
and "baxter_l_arm_traj.jsp".

xxx

Trajectories recorded in this fashion can be played back with the playfile reader,
`baxter_playfile_jointspace`
## Example usage
With Baxter (sim or real) running, cd to a trajectory appropriate to save recordings.
Be careful to rename "merry_r_arm_traj.jsp" to avoid overwriting previous recording.
When ready to record, start:
`rosrun baxter_playfile_reader baxter_record_trajectory`
enter "1", then move the arm in the desired trajectory (path and speed).
When done with recording, ctl-C.  The result will be in "merry_r_arm_traj.jsp".

To play back a joint-space trajectory file, start up the robot.  Start up the trajectory
interpolation action server:
`rosrun baxter_traj_streamer traj_interpolator_as`
In another terminal, cd to the directory containing the desired filename.
Start the playback using the desired filename (e.g., "merry_r_arm_traj.jsp").
`rosrun baxter_playfile_reader baxter_playfile_jointspace merry_r_arm_traj.jsp`

## Running tests/demos
    
