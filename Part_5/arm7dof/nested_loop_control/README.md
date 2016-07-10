# nested_loop_control
The inner-velocity-loop controller performs position control through a successive loop closure
technique.  Velocity commands are generated to emulate joint-by-joint mass-spring-damper behavior
based on a joint-position attractor.  Joint attractor positions are hard-coded initialized, but can
be specified via publication to topic "qdes_attractor_vec".  

## Example usage
Start up Gazebo:
`roslaunch gazebo_ros empty_world.launch`
Bring in the robot model with joint-velocity controllers:
`roslaunch arm7dof_model arm7dof_w_vel_controller.launch`
Start up the velocity-based, nested-loop position controller:
`rosrun nested_loop_control inner_vel_loop`
This will put the robot in a pose suitable for catching a falling mass.  The controller will respond
to published joint attractors.  An example is:
`rosrun nested_loop_control test_inner_vel_loop`
which will prompt for a joint number, amplitude and frequency, then command sinusoidal values to the
targeted joint.

This controller can be used as a type of position controller, and it illustrates steps towards
a Natural Admittance Controller.

    
