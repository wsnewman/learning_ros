# maps
dummy ROS package just to hold maps

##data acquistion process for map making
To make a map, first bag some data.  e.g., start up gazebo:
`roslaunch gazebo_ros empty_world.launch`

Then start up the mobot in the starting pen, along with supporting nodes:
`roslaunch mobot_urdf mobot_startup_open_loop.launch`

Record a bagfile that includes the LIDAR topic and /tf (for odometry).
Choose where this file is to be written (e.g., this directory) and choose a filename (e.g. mapData)
`rosbag record -O mapData /scan /tf`

Start the robot moving.  E.g, run a 3m square with:
`rosrun mobot_pub_des_state pub_des_state_path_client_3x3`

Stop recording after enough data is logged.

##conversion of bag data to a map
Kill gazebo (if it is still running).  Start a roscore.  From this (maps) directory, run:
`rosrun gmapping slam_gmapping scan:=/scan`
(if your robot uses a different LIDAR topic than "scan", e.g. /laser/scan, use: scan:=/laser/scan)

Playback the recorded bag file with, e.g.:
`rosbag play mapData.bag`

To watch the progress, start up rviz:
`rosrun rviz rviz`
and add a "map" display topic.  Can watch the map-building progress.

When the playback is done, save the computed map with a chosen map name:
`rosrun map_server map_saver -f newMap`






    
