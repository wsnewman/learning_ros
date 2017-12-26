# arm_motion_action

Define the action message used by arm_motion_interface (in action server) and generic_cartesian_planner/cart_motion_commander.
This action message defines function codes for various functional behaviors, as well as return-value codes for responses.
Different function codes may also require various parameter values, which are options within the goal message.

## Example usage
To use this goal message, include #include <arm_motion_action/arm_interfaceAction.h>

    
