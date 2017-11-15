//example_frame_inversion.cpp: 

#include<ros/ros.h>
#include <xform_utils/xform_utils.h>
using namespace std;


int main(int argc, char** argv) {
    ros::init(argc, argv, "example_frame_inversion"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    XformUtils xformUtils; //instantiate an object of XformUtils
    //geometry_msgs::Pose object_pose, gripper_pose;
    //geometry_msgs::PoseStamped gripper_pose_stamped;
    
//freenect driver:
//  world-> /camera_link -> /camera_depth_frame  -> /camera_depth_optical_frame
    
 
//get camera_link tf info from freenect and tf_echo:
// rosrun tf tf_echo camera_link camera_depth_optical_frame
//    - Translation: [0.000, -0.020, 0.000]
//    - Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
//            in RPY (radian) [-1.571, -0.000, -1.571]
//            in RPY (degree) [-90.000, -0.000, -90.000]
    
    Eigen::Affine3d affine_cdof_wrt_camera_link; //camera_depth_optical_frame = cdof
    geometry_msgs::Pose pose_cdof_wrt_camera_link;
    pose_cdof_wrt_camera_link.position.x = 0;
    pose_cdof_wrt_camera_link.position.y = -0.020;
    pose_cdof_wrt_camera_link.position.z = 0;   
    
    pose_cdof_wrt_camera_link.orientation.x = -0.5;    
    pose_cdof_wrt_camera_link.orientation.y = 0.5;
    pose_cdof_wrt_camera_link.orientation.z = -0.5;
    pose_cdof_wrt_camera_link.orientation.w = 0.5;
    
    //Eigen::Affine3d XformUtils::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    affine_cdof_wrt_camera_link = xformUtils.transformPoseToEigenAffine3d(pose_cdof_wrt_camera_link);
    ROS_INFO("affine_cdof_wrt_camera_link");
    xformUtils.printAffine(affine_cdof_wrt_camera_link);
    
    //enter some data to construct a frame to be inverted:
//A_plane_wrt_camera 
    Eigen::Vector3d x_axis, y_axis, z_axis, origin;
    Eigen::Matrix3d R_mat, R_mat_inv;    
    Eigen::Affine3d Affine_orig, Affine_inverted, affine_cdof_wrt_world;
// select a patch of points on the floor to get plane properties:
//might choose points so major axis is parallel to robot x-axis
//PCL: plane params of patch: -0.0770101 0.131265 0.988352 -1.70353
// A_plane_wrt_camera rotation:
//     0.99703 9.31323e-10   0.0770101
//   0.0101388   -0.991296   -0.131265
//   0.0763398    0.131656   -0.988352
// origin: -0.131189 0.223614  1.68369

    
    
//  origin: -0.0115117 0.295769  1.70508

 //   object_pose.position.x = -0.0115117;
 //   object_pose.position.y = 0.295769;
 //   object_pose.position.z = 1.70508;
    


// PCL: plane params of patch: 0.0067745 0.169288 0.985543 -1.72283
//A_plane_wrt_camera rotation:
//    0.999977 -1.16415e-10     -0.0067745
//   -0.00114686    -0.985566    -0.169288
//   -0.00667671     0.169291    -0.985543
// origin: 0.0116713 0.291654  1.69793
    
// A_plane_wrt_camera rotation:
//  0.999899          0 -0.0142403
//-0.00254161  -0.983944  -0.178462
//-0.0140116    0.17848  -0.983844
//origin: 0.0244295 0.306155   1.6878
    
//centroid:  0.705721 -0.121051   1.79549
//[ INFO] [1492448479.409710211]: major_axis: 0.051329, -0.988685, 0.140955
//[ INFO] [1492448479.409810251]: plane normal: 0.074957, -0.136929, -0.987741
//[ INFO] [1492448479.409872389]: plane dist = -1.704005
//[ INFO] [1492448479.409927911]: plane normal = (0.074957, -0.136929, -0.987741  
    
 //     0.99703 9.31323e-10   0.0770101
//   0.0101388   -0.991296   -0.131265
//   0.0763398    0.131656   -0.988352   
    origin<<-0.131189, 0.223614,  1.68369;    
    x_axis<< 0.99703, 0.0101388, 0.0763398;
    z_axis<< 0.077, -0.131265, -0.988352; //should be approx 0,0,-1
    y_axis= z_axis.cross(x_axis);
    
    R_mat.col(0) = x_axis;
    R_mat.col(1) = y_axis;
    R_mat.col(2) = z_axis;
    
    Affine_orig.linear()= R_mat;
    Affine_orig.translation() = origin;
    
    Affine_inverted = Affine_orig.inverse(); //this should be T_cdof/world
    
    affine_cdof_wrt_world = Affine_inverted;
    ROS_INFO("original affine (patch w/rt cdof): ");
    xformUtils.printAffine(Affine_orig);
    Eigen::Quaterniond quat(R_mat); //quaternion corresponding to given affine
    ROS_INFO("quaternion: x,y,z,w =  %f, %f, %f, %f",quat.x(),quat.y(),quat.z(),quat.w());    
    
   double qx,qy,qz,qw;    
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();
    qw = quat.w();        
    //ROS_INFO("qx,qy,qz,qw = %f, %f, %f, %f",qx,qy,qz,qw);
    tf::Quaternion tf_quat_orig(qx,qy,qz,qw); //a tf quaternion from Eigen quaternion
    ROS_INFO("corresponds to these RPY vals:");
    double roll,pitch,yaw;
    tf::Matrix3x3(tf_quat_orig).getRPY(roll, pitch, yaw);
    ROS_INFO("equiv RPY = %f %f %f ",roll,pitch,yaw); 
    
    
    ROS_INFO("inverted affine (cdof w/rt world): ");
    xformUtils.printAffine(affine_cdof_wrt_world);   
    R_mat_inv = affine_cdof_wrt_world.linear();
    Eigen::Quaterniond eig_quat(R_mat_inv); //quaternion corresponding to given affine
    ROS_INFO("affine_cdof_wrt_world quaternion: x,y,z,w =  %f, %f, %f, %f",eig_quat.x(),eig_quat.y(),eig_quat.z(),eig_quat.w());
    //convert to tf_quat:
    //Quaternion (const tfScalar &x, const tfScalar &y, const tfScalar &z, const tfScalar &w)
  
    qx = eig_quat.x();
    qy = eig_quat.y();
    qz = eig_quat.z();
    qw = eig_quat.w();        
    //ROS_INFO("qx,qy,qz,qw = %f, %f, %f, %f",qx,qy,qz,qw);
    tf::Quaternion tf_quat(qx,qy,qz,qw); //a tf quaternion from Eigen quaternion
    ROS_INFO("corresponds to these RPY vals:");
    //double roll,pitch,yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    ROS_INFO("equiv RPY = %f %f %f ",roll,pitch,yaw);      
    
    //if have cdof w/rt world, and have cdof w/rt camera_link, compute camera_link w/rt world
    // T_cdof/world = T_camera_link/world*T_cdof/camera_link;
    // T_camera_link/world = T_cdof/world*inv(T_cdof/camera_link)
    Eigen::Affine3d affine_camera_link_wrt_world;
    affine_camera_link_wrt_world = affine_cdof_wrt_world*affine_cdof_wrt_camera_link.inverse();
    ROS_INFO("inferred camera_link with respect to world: ");
    xformUtils.printAffine(affine_camera_link_wrt_world);
    Eigen::Matrix3d R_mat_cam_link_wrt_world;
    R_mat_cam_link_wrt_world = affine_camera_link_wrt_world.linear();
    Eigen::Quaterniond eig_quat_cam_link(R_mat_cam_link_wrt_world); //quaternion corresponding to given affine
    ROS_INFO("camera_link wrt world quaternion: x,y,z,w =  %f, %f, %f, %f",eig_quat_cam_link.x(),eig_quat_cam_link.y(),eig_quat_cam_link.z(),eig_quat_cam_link.w());
}
