# mobot_mapping
Illustrate the mapping process with Gazebo

to use mobot to map starting pen, start up Gazebo with:
`roslaunch mobot_urdf mobot_in_pen.launch`

start of gmapping, rviz and teleop_twist_keyboard with:
`roslaunch mobot_mapping mobot_startup_gmapping.launch`

Drive around to build up a map.  Then save the map with:
`rosrun map_server map_saver -f startPenMap`
which will save a map called "startPenMap" in the directory from which the above command is run.

Then load the map with:
`rosrun map_server map_server startPenMap.yaml`

With the map loaded, one can localize with amcl by running:
`rosrun amcl amcl`

## Example usage

## Running tests/demos
    
