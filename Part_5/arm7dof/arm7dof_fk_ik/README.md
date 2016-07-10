# arm7dof_fk_ik
Contains kinematics library arm7dof_fk_ik.  Use of these functions is illustrated by the
test functions arm7dof_fk_ik_test_main and arm7dof_fk_ik_test_main2.

## Example usage


## Running tests/demos
`roslaunch gazebo_ros empty_world.launch`
`roslaunch arm7dof_model arm7dof_w_pos_controller.launch`
`rosrun rqt_gui rqt_gui` (to command joint angles)
`rosrun arm7dof_fk_ik arm7dof_fk_ik_test_main`, or:
`rosrun arm7dof_fk_ik arm7dof_fk_ik_test_main2`    