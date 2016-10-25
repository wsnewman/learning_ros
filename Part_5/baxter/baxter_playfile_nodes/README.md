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
When done with recording, ctl-C.  The results will be in "baxter_r_arm_traj.jsp" and "baxter_l_arm_traj.jsp".

To play back joint-space trajectory files, start up the robot, wait for it to finish
booting up, then enable the robot, start the trajectory action servers with:
`roslaunch baxter_launch_files baxter_playfile_nodes.launch`

This launch file also invokes motion of the arms to a pre-pose position, and it starts up
a multi-trajectory node that accepts codes corresponding to pre-recorded playfiles (baxter_multitraj_player: see below).

In another terminal, cd to the directory containing the desired trajectory filename(s).
Run the playback node, with command-line arguments for the right-arm trajectory and left-arm trajectory.
(If only 1 filename is provided, it will be interpreted and executed as the right-arm file). E.g.:
`rosrun baxter_playfile_nodes baxter_playback baxter_r_arm_traj.jsp`
(to move just the right arm).  Or,
`rosrun baxter_playfile_nodes baxter_playback baxter_r_arm_traj.jsp baxter_l_arm_traj.jsp`
to move both arms.

The node baxter_multitraj_player is an alternative playfile node, which can be run as:
`rosrun baxter_playfile_nodes baxter_multitraj_player`
This node can be run from any directory, but it will always look for pre-recorded jsp playfiles in
the baxter_playfile_nodes package directory.  Consequently, it can be started conveniently
within a launch file.  (see baxter_launch_files/launch/baxter_playfile_nodes.launch, which simplifies
launching nodes needed to execute playfiles).  The multitraj player listens on topic playfile_codes
for a code number from 0 through 6, and maps these onto execution of corresponding existing playfiles.
For example, running:
`rostopic pub playfile_codes std_msgs/UInt32 2`
invokes execution of the playfile ``shy.jsp''  With the multitraj player running, one can invoke
pre-recorded moves from other programs by merely publishing a playfile code to the playfile_codes
topic.



    
