# ur10_launch
To bring up the ur10 robot on a pedestal, run:
`roslaunch ur10_launch ur10.launch` 
Add table and block:
`roslaunch exmpl_models add_table_and_block.launch`

## Example usage
To run a version of the UR10, on a pedestal, with a simulated vacuum gripper, do:
`roslaunch ur10_launch ur10_w_gripper.launch`

Next, add a table and block and start the nodes needed for the object-grabber service:
`roslaunch ur10_launch ur10_object_grabber_nodes.launch`

Run an action client of object grabber:
`rosrun object_grabber example_object_grabber_action_client`

    
