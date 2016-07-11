# arm7dof_motion_plans
ROS package to simply store motion plan files for arm7dof. 
motion_plan.pth is a path file (no timing), and motion_plan.trj is a trajectory file (includes timing)

## Example usage
rosrun arm7dof_traj_as  arm7dof_playfile_path  -file $(rospack find arm7dof_motion_plans)/arm7dof_poses.dat
rosrun arm7dof_traj_as  arm7dof_playfile_path arm7dof_poses.pth    
