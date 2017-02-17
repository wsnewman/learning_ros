# example_nav_plugin
This package illustrates how to create a plug-in to replace the default base_local_planner within
the nav-stack.  

## Example usage
start up a gazebo simulation of the mobot in the starting pen with:
(optirun) `roslaunch mobot_urdf mobot_in_pen.launch`

start up various nodes, including move_base, with specification to use the local planner
created in this package:

`roslaunch example_nav_plugin mobot_w_minimal_plugin.launch` 

In this trivial example, one initiates global planning by specifying a navigation goal (which can
be done from the rviz toolbar, using the 2D NavGoal tool).  The computed global plan is displayed
in rviz.  This initiates local planning.  However, for the minimal_nav_plugin, the result is
simply to command a fixed forward speed and yaw rate, imposed for 5 seconds of motion.

## plugin design steps
This minimal plug-in example requires several steps.  The package.xml file contains the lines:
  <export>
     <!-- the following line refers to another xml file, in which some
     info re/ the new plugin library is defined  -->
    <nav_core plugin="${prefix}/nav_planner_plugin.xml" />
  </export>
which points to an additional xml file used to load the new plugin library.

The additional file, nav_planner_plugin.xml, contains:
<library path="lib/libminimal_nav_plugin">
    <!--example_nav_plugin is the package name, and MinimalPlanner is the class name  -->
    <!--the base class is nav_core::BaseLocalPlanner; generically, build on this to write a new
       local planner  -->
	<class name="example_nav_plugin/MinimalPlanner" type="MinimalPlanner" base_class_type="nav_core::BaseLocalPlanner">
		<description>Example local planner plugin- causes the robot to move in a CCW circle.</description>
	</class>
</library>
which specifies that the new plugin is derived from the nav_core's BaseLocalPlanner.  It notes that
the compiled library is called libminimal_nav_plugin, which is a name derived by prepending "lib" to
the library name specified in the CMakeLists.txt file of this package.  The plugin contains
a new class, and this class's name is specified using the package_name/class_name
(example_nav_plugin/MinimalPlanner).  

In the CMakeLists.txt file, it is specified that a new library is to be created:
add_library(minimal_nav_plugin src/minimal_nav_plugin.cpp)

The library is compiled from source code minimal_nav_plugin.cpp, and the library name will
be minimal_nav_plugin (from which the compiler will create libminimal_nav_plugin.so).
minimal_nav_plugin must provide alternative functions to override the defaults. 


The source code in minimal_nav_plugin.cpp provides the implementation of the class MinimalPlanner,
which is defined in the corresponding header file minimal_nav_plugin.h.  Importantly, the
function computeVelocityCommands() must be provided, in which values of cmd_vel are specified
to drive the robot.  In the present case, these are simply hard-coded constants.  More generally,
cmd_vel would be based on sensors and planning.

To use the plugin, it is also necessary that move_base be launched with the option to use
the new local-planner plugin.  The launch file mobot_w_minimal_plugin.launch brings up
supporting nodes for the mobot (as previously), but the new launch file also contains the line:
    <param name="base_local_planner" value="example_nav_plugin/MinimalPlanner"/>
which causes move_base to use the new local planner plugin.


    
