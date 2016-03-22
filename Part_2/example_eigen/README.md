# example_eigen
Example program to illustrate use of Eigen methods.  This node creates random, nearly-planar data, then applies a covariance
eigenvalue/eigenvector method to find the best-fit plane parameters.

## Example usage
With roscore running, run:
`rosrun example_eigen example_eigen_plane_fit`
In the code, can change the values of the prescribed plane normal and offset and synthetic data noise, then examine the 
resulting identified parameters.  For zero noise, the result should be virtually perfect. 
    