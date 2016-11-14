#ifndef OBJECT_MANIPULATION_PROPERTIES_H
#define	OBJECT_MANIPULATION_PROPERTIES_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h> //these could be handy in the future
#include <geometry_msgs/PoseStamped.h>
#include <object_manipulation_properties/gripper_ID_codes.h>
#include <object_manipulation_properties/object_ID_codes.h>


class ObjectManipulationProperties {
private:
    XformUtils xformUtils;

public:
    ObjectManipulationProperties(void);
    bool get_object_info(int object_id, Eigen::Affine3d &grasp_transform, double &approach_dist, double &gripper_close_test);
};
#endif
