# interactive_marker_node

Interactive-marker node.  This node creates an interactive marker, visualizable in rviz.
Topic name is: rt_hand_marker.  In rviz, select this as an interactive marker subscribed to
topic: rt_hand_marker/update.

This node initializes the pose of the interactive marker, then listens for changes due to 
user interaction.  It keeps track of current pose.

Also provides a service called "/IM6DofSvc" which uses services messages of type:
ImNodeSvcMsg (defined in this package).  IM service requests contain a command mode.  If mode is
GET_CURRENT_MARKER_POSE, then the service response contains the current pose of the marker.

If the command mode is SET_NEW_MARKER_POSE, then the poseStamped_IM_desired field of the
request must also be filled in, specifying the desired marker pose.  The service will then
move the marker to this pose and save and return the new pose values.

## Example usage
Run this node together with roscore and rviz.  
`rosrun rviz rviz`, and add an interactive-marker item, subscribed to rt_hand_marker/update.
Set the fixed frame to "world."  

Start the interactive-marker node with:
`rosrun interactive_marker_node interactive_marker_node` 

To test the available service, run:
`rosrun  interactive_marker_node im_6dof_svc_client_test`  

This gets the current marker pose, then causes the pose to move up by 0.1m.  One can also move the marker interactively and still use IM node services to get or change the marker pose.

    
