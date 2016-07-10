# irb120_ik
Forward and inverse kinematics functions for the ABB IRB120 arm.  This package creates
a library of FK and IK functions.  The IK method exploits the spherical-wrist condition.
From 0 to 8 solutions are returned, given a desired wrist-flange pose.

## Example usage
See nodes irb120_reachability_from_above.cpp and irb120_fk_ik_test.cpp for example nodes using this library. 
## Running tests/demos
irb120_fk_ik_test.cpp

    
