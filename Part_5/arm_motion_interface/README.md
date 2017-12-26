# arm_motion_interface
This library is the counterpart to the cartesian_motion_commander library.
The arm_motion_interface library defines a class ArmMotionInterface.
This class starts up an action server, "cartMoveActionServer", which communicates via action messages defined in arm_motion_action.
Goal messages include (at a minimum) an action code (as defined in arm_motion_action/arm_interface.action).
Depending on the action code (function) specified, additional arguments may also be required within additional fields
of the goal message.

The ArmMotionInterface switches to different functions, depending on the function code.  For planning functions, this class
relies on implementations in generic_cartesian_planner.

## Example usage
An action client uses the cartesian_motion_commander (which defines class CartMotionCommander).  By invoking functions
within this class, it populates and sends goal messages (as defined in arm_motion_action/arm_interface.action) to
action server "cartMoveActionServer".  

The intent is that the action clients are robot agnostic.
A corresponding action server must be customized for a given robot.  The arm_motion_interface helps with code re-use for
different robot-specific action servers.

See example in irb120_planner/src/irb120_cart_move_as.cpp

## Running tests/demos
    
