This package illustates how to create a plug-in of nav_core that substitutes an alternative
base_local_planner.  The simple demo merely commands a 5-second motion with fixed speed/spin
each time a new global plan is received (after the 5-second motion is completed).

This simple test plugin does not use localization, does not perform steering, and in fact
ignores the global plan.  However, it could be upgraded to perform these functions.


