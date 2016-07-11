# joint_space_planner
Provides a library to perform dynamic programming on a feedforward network to obtain the optimal (min-cost)
path through the network.  Useful for choosing among null-space options to compute a desirable joint-space
path corresponding to a specified Cartesian path.

## Example usage
See the cartesian_planner package for example use.  The joint-space planner must be instantiated with
constructor arguments consisting of a network (a vector of vectors of joint-space poses) and penalty weights
(relative importance of delta-angle motions of each joint).


    
