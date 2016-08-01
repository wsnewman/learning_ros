# simple_camera_model
A simple Gazebo model containing a color camera.  Used to illustrate camera operations.
The camera model is contained within a gazebo plug-in in "simple_camera_model.xacro".

A launch file, "simple_camera_simu.launch", brings up the gazebo simulator, spawns the camera model,
and also spawns a model of a red block, within view of the camera.

To interpret image data from this model with respect to the world frame, one additionally
needs to publish a static transform between
"camera_link" and "camera_optical_frame"  (not done here; see, e.g., launch file  "find_red_pixels.launch"
in package "example_opencv")  

## Example usage
`roslaunch simple_camera_model simple_camera_simu.launch`

`rosrun image_view image_view image:=/simple_camera/image_raw`

An example that uses open_cv to process images from this camera model is in package
"example_opencv".  E.g., launch:
`roslaunch example_opencv find_red_pixels.launch`

    
