# arm7dof_motion_plans
ROS package to simply store motion plan files for arm7dof. 
motion_plan.path is a path file (no timing), and motion_plan.trj is a trajectory file (includes timing)

## Example usage
`roslaunch gazebo_ros empty_world.launch`
`roslaunch arm7dof_model arm7dof_w_vel_controller.launch`
`rosrun nested_loop_control inner_vel_loop`
`rosrun arm7dof_traj_as arm7dof_traj_as`

from this directory, run:
`rosrun arm7dof_traj_as  arm7dof_playfile_path fname.path`    
