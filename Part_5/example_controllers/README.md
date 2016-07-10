# example_controllers
Simple 1-DOF robot with a prismatic joint is in model "prismatic_1dof_robot_description_w_jnt_ctl.xacro".
Illustrates 3 types of controller: joint position control (plug-in), which uses control parameters in
subdirectory "control_config", file "one_dof_pos_ctl_params.yaml".  The position-controlled robot is launched with:
`roslaunch example_controllers prismatic_1dof_robot_w_jnt_pos_ctl.launch` 

Joint velocity control illustration uses the same URDF, but is launched with:
`roslaunch example_controllers prismatic_1dof_robot_w_jnt_vel_ctl.launch`

Force control is also illustrated, using velocity control as an inner loop.  This controller
requires a model of a force sensor, which is included in the robot URDF.

rosrun rqt_gui rqt_gui
rosrun example_controllers sine

  created example_force_control/prismatic_1dof_robot_description_w_jnt_ctl.urdf
  has effortJointInterface
  
launch file: example_force_control/prismatic_1dof_robot_w_jnt_ctl.launch
  references config in: .../control_config/one_dof_vel_ctl_params.yaml
  which sets pure velocity-control proportional gain
  
can test the velocity-control interface as follows:
`roslaunch gazebo_ros empty_world.launch` 
`roslaunch example_force_control prismatic_1dof_robot_w_jnt_ctl.launch` 
`rosrun example_force_control sine_vel_commander` 

To test an NAC controller, instead of sine_vel_commander, run:
`rosrun example_force_control nac_controller`
Then robot behaves like a spring/damper piston.  

Can, e.g., import a model and "drop" it on the robot (virtual piston) to observe transient response of NAC.

Drop a cylindrical weight by running:
`roslaunch example_force_control add_weight.launch`
(may need to adjust gravity, f_sat, K_virt, ...)


  view with: 
  `rqt_plot`
  test tuning w/ 
  `rosrun rqt_reconfigure rqt_reconfigure`
  

## Example usage

## Running tests/demos
    
