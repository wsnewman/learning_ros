# cartesian_planner
This package includes separate, but very similar Cartesian planners for Baxter and for arm7dof.
There are multiple cartesian-plan options, including:

*specify start and end poses w/rt base.  Only orientation of end pose will be considered; orientation of start pose is ignored;
planned motion keeps orientation constant while moving in a straight line from start position to end position.

*specify start as a q_vec, and goal as a Cartesian pose (w/rt base).  Orientation of goal pose will be obtained quickly,
then preserved through the linear move.

*specify start as a q_vec, and desired delta-p Cartesian motion while holding R fixed at initial orientation

## Example usage
The node "example_arm7dof_cart_path_planner_main.cpp" shows how to use the Cartesian planner, which relies 
on support from a corresponding fk_ik library and from the generic joint-space planner.

Running this node will produce an output file "arm7dof_poses.dat" comprised of joint-space poses that produce Cartesian motion with
the tool flange orientation constant.   The desired motion is specified hard-coded in the main program
to maintain orientation of the tool flange pointing up while translating at y-desired, z-desired from x-start to x-end.
These values can be edited to test alternative motions.

## Running tests/demos
`rosrun  cartesian_planner example_arm7dof_cart_path_planner_main`    
