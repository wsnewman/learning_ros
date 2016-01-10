# example_parameter_server
This package illustrates how to read parameters programmatically from the parameter server.

## Example usage
Load parameters from a yaml file via a launch file with:
`roslaunch example_parameter_server load_gains.launch`
Then run the example node with:
`rosrun example_parameter_server read_param_from_node`
to demonstrate that the node has obtained the desired parameters from the parameter server.

