# example_rviz_marker
This node illustrates how to display markers in rviz. 
The service rviz_marker_svc expects a floating-point number, which is uses to 
set the height of a horizontal plane of markers.

The node triad_display subscribes to topic "triad_display_pose" to receive stamped poses, then
constructs a triad of markers to display 3 axes corresponding to the received pose.
To view, add a Marker item in rviz and set the topic to /triad_display

## Example usage
Run this node together with roscore and rviz.  
`rosrun rviz rviz`
`rosrun example_rviz_marker example_rviz_marker` 
in the rviz displays, add a marker item, and set it to subscribe to example_marker_topic.
A horizontal plane of red markers will be displayed at height zero.  Interactively, change the height, e.g.:
`rosservice call rviz_marker_svc 1.0`
to generate the markers at height 1.0 m.

For the triad display:
`rosrun rviz rviz`
add marker item, topic /triad_display
`rosrun example_rviz_marker triad_display`
Start publishing test poses with the test node:
`rosrun example_rviz_marker triad_display_test_node`

Similarly, illustration of painting line segments on the ground plane can be run with:
`rosrun example_rviz_marker lines_display`
`rosrun example_rviz_marker linesegs_display_test_node`

    
