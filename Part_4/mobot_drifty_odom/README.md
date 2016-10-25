# mobot_drifty_odom
Publishes a flawed (drifty) odom on topic drifty_odom.  The robot model is flawed
by having one wheel slightly larger radius than the other.  Consequently, integration
of odometry leads to arbitrarily large accumulated error.  This is useful for evaluating
error tolerance when integrating with additional absolution information, e.g. from LIDAR 
or GPS.

    