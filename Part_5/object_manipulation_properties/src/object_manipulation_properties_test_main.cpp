#include <object_manipulation_properties/object_manipulation_properties.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_manip_prop_test_main"); // name this node 
    ObjectManipulationProperties objectManipulationProperties;
    
    Eigen::Affine3d grasp_transform;
    double approach_dist, gripper_test_val;
    int object = TOY_BLOCK_ID;
    
    cout<<"enter object ID code (try 1000): ";
    cin>>object;
    
    if (objectManipulationProperties.get_object_info(object,grasp_transform,approach_dist,gripper_test_val)) {
        cout<<"approach_dist = "<<approach_dist<<endl;
        cout<<"gripper test val= "<<gripper_test_val<<endl;
        cout<<"transform origin: "<<grasp_transform.translation().transpose()<<endl;
    }
    else {
        ROS_WARN("object ID not recognized");
    }
}
