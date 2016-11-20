# cwru_ariac_launch

Launch files for subset of NIST/Ariac system.

## Example usage
Bring up UR10 robot, on pedestal, with sticky-fingers vacuum gripper emulation:
`roslaunch ur10_launch ur10_w_gripper.launch`

Launch associated nodes for object-grabber operation w/ simulated
vacuum gripper.  Spawn a table and 4 part bins.
`roslaunch cwru_ariac_launch cwru_ariac.launch`

Add parts to bins:
`roslaunch ariac_models add_parts.launch`

Start an action client that moves one block to the table:
`rosrun object_grabber block_grabber_action_client`

OR,
run a part-fetcher service:
`rosrun part_fetcher part_fetcher_soln`

and a corresponding client:
`rosrun part_fetcher example_part_fetcher_client`

    
