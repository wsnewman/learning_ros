# example_opencv
Simple examples of using open_cv with ROS.  Nodes here subscribe to images on the topic
"simple_camera/image_raw".  Emulated camera images are published to this topic via the
model in package "simple_camera_model".  One can start gazebo simulation of a simple,
ideal, downward-pointing camera model by launching:
`roslaunch simple_camera_model simple_camera_simu.launch`

The node "find_red_pixels" looks for the image topic "simple_camera/image_raw"; 
searches for (sufficiently) red pixels;
converts (sufficiently) red pixels to white and all other pixels to black;
computes the centroid of red pixels and displays the centroid as a blue square on
the output image; displays the processed image in the Open-CV viewer, and also
publishes the processed image on topic "/image_converter/output_video".  

With the gazebo simulation running, start this example node with:
`roslaunch example_opencv find_red_pixels.launch`

This launch file starts the node "find_red_pixels."  In addition, it starts up
the ROS node "image_viewer/image_viewer" and remaps the input topic of this node
to "simple_camera/image_raw".  This results in a display window showing the simulated camera view.

In addition, this launch file starts up a static-transform publisher node that publishes a static transform 
between "camera_link" and "camera_optical_frame".  This transform completes the kinematic chain from
the cameras image plane to the world frame, which enables ROS to interpret spatial relationships.

The "find_red_pixels" node prompts the user to enter a parameter for a redness ratio (e.g., enter "10"),
then begins to interpret images.  The red block can be moved in gazebo, and the
resulting (raw) image will be updated in an "image_view" window while the processed image
is updated in the open-cv viewer window.

Additionally, one can view the ROS publications of processed images with:
`rosrun imagview image_view image:=/image_converter/output_video`
The display via this ROS node will be identical to the open-cv viewer.  This validates
that the ROS publication of processed images is being performed successfully.

Example corner detection:
`roslaunch simple_camera_model simple_camera_simu_w_checkerboard.launch` 
`rosrun example_opencv harris_corners`



