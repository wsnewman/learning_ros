#ifndef OBJECT_MANIPULATION_PROPERTIES_H
#define	OBJECT_MANIPULATION_PROPERTIES_H

#include<ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h> //these could be handy in the future
#include <geometry_msgs/PoseStamped.h>


const int TOY_BLOCK_ID=1;
//const int ANOTHER_OBJECT_ID= ...

class ObjectManipulationProperties {
private:
  XformUtils xformUtils;
  
public:
ObjectManipulationProperties(void);    
bool get_object_info(int object_id, Eigen::Affine3d &grasp_transform, double &approach_dist, double &gripper_close_test);
};
#endif
