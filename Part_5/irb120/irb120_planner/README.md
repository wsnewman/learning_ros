# irb120_planner
This package contains irb120_cart_move_as, which presents an action server "cartMoveActionServer".
The intent is to re-use code for Cartesian-motion planning and present a generic robot interface to action clients, so 
a robot model can be substituted without changing higher-level code.

Some robot-specific code is necessary within the cartesian-motion action server.  
The following must be done to customize the Cartesian-motion action server to a particular robot.

irb120_fk_ik/irb120_kinematics.h: include this file in irb120_cart_move_as.cpp to specify desired robot fk/ik.  Also list
corresponding package dependency in package.xml

robot_specfic_names.h:  this file must be created/edited to specify key names, including:
*the joint names (in order expected by kinematics routines)
*the base_link name consistent with the URDF
*the name of the topic to which joint states will be published
*the name of the topic to which joint trajectory commands may be published to move the robot
The file "robot_specfic_names.h" is included in irb120_cart_move_as.cpp

robot_specific_fk_ik_mappings.h: this file must be created/edited to refer to robot-specific kinematics functions
Edit the following:
*include the appropriate robot kinematics header
*modify lines in derived classes such as irb120_fwd_solver_.fwd_kin_solve(q_vec) to refer to the actual fk/ik function names that
will override the virtual function names in fk_ik_virtual.h

With changes to these two robot-specific header files, the action server will compile to be specialized to the target robot

## Example usage

## Running tests/demos
    
