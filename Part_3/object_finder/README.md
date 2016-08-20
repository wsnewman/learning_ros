# object_finder
This package is a "shell" for development of an object-finder action service.
In the goal message, specify a code corresponding to a known object type.
Optionally, specify if it is known that the object is on a horizontal surface of specified height.
Optionally, specify dimensions (for generic shapes).

The response will contain a return code, e.g. for "object found", "object type not recognized",
or "object not found".  If object is found, also fill in the poseStamped for the found object.

Current version of server ONLY responds to inquiries re/ a Coke can on surface of known height,
and it responds with a hard-coded pose.  Need to fill in actual PCL processing.
(a pcl_utils object is instantiated to assist this).

Specifically, replace the member method:
bool ObjectFinder::find_upright_coke_can(float surface_height,geometry_msgs::PoseStamped &object_pose)


## Example usage
`rosrun object_finder object_finder_as`
`rosrun object_finder example_object_finder_action_client`

or:
`roslaunch worlds play_pen_world.launch`
`roslaunch simple_camera_model kinect_simu2.launch`
`roslaunch exmpl_models add_toy_block.launch`
`rosrun object_finder object_finder_as`
`rosrun  example_rviz_marker triad_display`
`rosrun object_finder example_object_finder_action_client`

## Running tests/demos
    
