#ifndef GRIPPER_ID_CODES_H
#define	GRIPPER_ID_CODES_H

namespace GripperIdCodes {
const int STICKY_FINGERS = 1; //assigned to a frame to emulate vacuum or electromagnetic gripper in Gazebo sim
const int YALE_GRIPPER_MODEL_T = 2; //Yale gripper, model-T
const int ARIAC_VACUUM_GRIPPER = 3; //vacuum gripper, as modeled for Ariac/NIST Gazebo simulation
const int RETHINK_ELECTRIC_GRIPPER_RT = 4; //model of electric gripper w/ rt-hand fingers as specified in Baxter simu
const int DUMMY_VACUUM_GRIPPER = 5; //assign to flange of UR10
//const int ANOTHER_OBJECT_ID= ...
}

#endif
