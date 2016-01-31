# example_rviz_marker
This node illustrates how to display markers in rviz. 
The service rviz_marker_svc expects a floating-point number, which is uses to 
set the height of a horizontal plane of markers.

## Example usage
Run this node together with roscore and rviz.  
`rosrun rviz rviz`
in the rviz displays, add a marker item, and set it to subscribe to example_marker_topic.
A horizontal plane of red markers will be displayed at height zero.  Interactively, change the height, e.g.:
`rosservice call rviz_marker_svc 1.0`
to generate the markers at height 1.0 m.

    
