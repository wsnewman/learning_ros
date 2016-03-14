# example_tf_listener

This code illustrates use of a transform listener.  This node requires that a transform from map to base_frame is being published.
This node simply prints out the x,y and yaw of a robot in map coordinates.

## Example usage
`rosrun example_tf_listener example_tf_listener`
Requires that some node publishers a transform from map to base_frame on the tf topic.
This can be accomplished, e.g. with:
gazebo (e.g. with the "starting pen" model in the world),
a corresponding map (e.g., startingPenMap, contained in cwru_urdf, and published by the map server),
cwruBot (from package cwru_urdf),
an amcl node (subscribing to LIDAR data)

AMCL will require some motion from the robot to publish updates of robot pose estimates in the map.

## Running tests/demos
    