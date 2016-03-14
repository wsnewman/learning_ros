//example_tf_listener_fncs.cpp:
//wsn, March 2016
//implementation of member functions of DemoTfListener class

#include "example_tf_listener.h"
using namespace std;

//constructor: don't need nodehandle here, but could be handy if want to add a subscriber
DemoTfListener::DemoTfListener(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    ROS_INFO("in class constructor of DemoTfListener");
    tfListener_ = new tf::TransformListener;  //create a transform listener and assign its pointer
    //here, the tfListener_ is a pointer to this object, so must use -> instead of "." operator
    //somewhat more complex than creating a tf_listener in "main()", but illustrates how
    // to instantiate a tf_listener within a class
    
    // wait to start receiving valid tf transforms between base_link and link2:
    // this example is specific to our mobot, which has a base_link and a link2
    // lookupTransform will through errors until a valid chain has been found from target to source frames
    bool tferr=true;
    ROS_INFO("waiting for tf between link2 and base_link...");
    tf::StampedTransform tfLink2WrtBaseLink; 
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
                tfListener_->lookupTransform("base_link", "link2", ros::Time(0), tfLink2WrtBaseLink);
            } catch(tf::TransformException &exception) {
                ROS_WARN("%s; retrying...", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    // from now on, tfListener will keep track of transforms; do NOT need ros::spin(), since
    // tf_listener gets spawned as a separate thread
}


//some conversion utilities:
//getting a transform from a stamped transform is trickier than expected--there is not "get" fnc for transform
tf::Transform DemoTfListener::get_tf_from_stamped_tf(tf::StampedTransform sTf) {
   tf::Transform tf(sTf.getBasis(),sTf.getOrigin()); //construct a transform using elements of sTf
   return tf;
}

//a transform describes the position and orientation of a frame w/rt a reference frame
//a Pose is a position and orientation of a frame of interest w/rt a reference frame
// can re-interpret a transform as a named pose using this example function
// the PoseStamped also has a header, which includes naming the reference frame
geometry_msgs::PoseStamped DemoTfListener::get_pose_from_transform(tf::StampedTransform tf) {
  //clumsy conversions--points, vectors and quaternions are different data types in tf vs geometry_msgs
  geometry_msgs::PoseStamped stPose;
  geometry_msgs::Quaternion quat;  //geometry_msgs object for quaternion
  tf::Quaternion tfQuat; // tf library object for quaternion
  tfQuat = tf.getRotation(); // member fnc to extract the quaternion from a transform
  quat.x = tfQuat.x(); // copy the data from tf-style quaternion to geometry_msgs-style quaternion
  quat.y = tfQuat.y();
  quat.z = tfQuat.z();
  quat.w = tfQuat.w();  
  stPose.pose.orientation = quat; //set the orientation of our PoseStamped object from result
  
  // now do the same for the origin--equivalently, vector from parent to child frame 
  tf::Vector3 tfVec;  //tf-library type
  geometry_msgs::Point pt; //equivalent geometry_msgs type
  tfVec = tf.getOrigin(); // extract the vector from parent to child from transform
  pt.x = tfVec.getX(); //copy the components into geometry_msgs type
  pt.y = tfVec.getY();
  pt.z = tfVec.getZ();  
  stPose.pose.position= pt; //and use this compatible type to set the position of the PoseStamped
  stPose.header.frame_id = tf.frame_id_; //the pose is expressed w/rt this reference frame
  stPose.header.stamp = tf.stamp_; // preserve the time stamp of the original transform
  return stPose;
}

//a function to multiply two stamped transforms;  this function checks to make sure the
// multiplication is logical, e.g.: T_B/A * T_C/B = T_C/A
// returns false if the two frames are inconsistent as sequential transforms
// returns true if consistent A_stf and B_stf transforms, and returns result of multiply in C_stf
// The reference frame and child frame are populated in C_stf accordingly
bool DemoTfListener::multiply_stamped_tfs(tf::StampedTransform A_stf, 
        tf::StampedTransform B_stf, tf::StampedTransform &C_stf) {
   tf::Transform A,B,C; //simple transforms--not stamped
  std::string str1 (A_stf.child_frame_id_); //want to compare strings to check consistency
  std::string str2 (B_stf.frame_id_);
  if (str1.compare(str2) != 0) { //SHOULD get that child frame of A is parent frame of B
      std::cout<<"can't multiply transforms; mismatched frames"<<endl;
    std::cout << str1 << " is not " << str2 << '\n'; 
    return false;
  }
   //if here, the named frames are logically consistent
   A = get_tf_from_stamped_tf(A_stf); // get the transform from the stamped transform
   B = get_tf_from_stamped_tf(B_stf);
   C = A*B; //multiplication is defined for transforms 
   C_stf.frame_id_ = A_stf.frame_id_; //assign appropriate parent and child frames to result
   C_stf.child_frame_id_ = B_stf.child_frame_id_;
   C_stf.setOrigin(C.getOrigin()); //populate the origin and orientation of the result
   C_stf.setBasis(C.getBasis());
   C_stf.stamp_ = ros::Time::now(); //assign the time stamp to current time; 
     // alternatively, could assign this to the OLDER of A or B transforms
   return true; //if got here, the multiplication is valid
}

//fnc to print components of a transform
void DemoTfListener::printTf(tf::Transform tf) {
    tf::Vector3 tfVec;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;
    tfVec = tf.getOrigin();
    cout<<"vector from reference frame to to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
    tfR = tf.getBasis();
    cout<<"orientation of child frame w/rt reference frame: "<<endl;
    tfVec = tfR.getRow(0);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
    tfVec = tfR.getRow(1);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;    
    tfVec = tfR.getRow(2);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl; 
    quat = tf.getRotation();
    cout<<"quaternion: " <<quat.x()<<", "<<quat.y()<<", "
            <<quat.z()<<", "<<quat.w()<<endl;   
}

//fnc to print components of a stamped transform
void DemoTfListener::printStampedTf(tf::StampedTransform sTf){
    tf::Transform tf;
    cout<<"frame_id: "<<sTf.frame_id_<<endl;
    cout<<"child_frame_id: "<<sTf.child_frame_id_<<endl; 
    tf = get_tf_from_stamped_tf(sTf); //extract the tf from the stamped tf  
    printTf(tf); //and print its components      
    }

//fnc to print components of a stamped pose
void DemoTfListener::printStampedPose(geometry_msgs::PoseStamped stPose){
    cout<<"frame id = "<<stPose.header.frame_id<<endl;
    cout<<"origin: "<<stPose.pose.position.x<<", "<<stPose.pose.position.y<<", "<<stPose.pose.position.z<<endl;
    cout<<"quaternion: "<<stPose.pose.orientation.x<<", "<<stPose.pose.orientation.y<<", "
            <<stPose.pose.orientation.z<<", "<<stPose.pose.orientation.w<<endl;
}
