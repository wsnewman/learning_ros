# baxter_fk_ik
This package contains a library of forward/inverse kinematics functions for the Baxter robot.  (Inverse kinematics for
the left arm is not complete).

An illustration of the use of this library is in the example program "baxter_reachability_from_above.cpp".  The baxter
fk/ik library is also used in nodes in the package "cartesian_planner".

## Example usage
The reachability node does not require the Baxter simulator to be running.  A simple roscore is adequate.  With
a roscore running, run:
`rosrun baxter_fk_ik baxter_reachability_from_above`
This will create an output file "reachable_x_y", which is a readable text file.  The x,y coordinates can be plotted
to view the poses that are reachable from above at the elevation specified in "baxter_reachability_from_above.cpp".

## Running tests/demos
    