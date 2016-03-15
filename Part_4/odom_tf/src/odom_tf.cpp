//odom_tf.cpp:
//wsn, March 2016
//illustrative node to show use of tf listener to reconcile odom and map frames
// w/rt base_link

// this header incorporates all the necessary #include files and defines the class "OdomTf"
#include "odom_tf.h"
using namespace std;

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "OdomTf_node"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    tf::StampedTransform stfBaseLinkWrtOdom; //base link w/rt odom frame; get this from tf; 
    tf::StampedTransform stfOdomWrtMap; //odom frame w/rt map frame; get this from tf, published by amcl
    tf::StampedTransform stfBaseLinkWrtMap; //base link w/rt map frame; compute this
    tf::StampedTransform stfBaseLinkWrtDriftyOdom; //base link w/rt drifty odom frame; get this from callback of odomTf 
    ROS_INFO("main: instantiating an object of type OdomTf");
    OdomTf odomTf(&nh);  //instantiate an OdomTf object and pass in pointer to nodehandle for constructor to use
    

    ROS_INFO:("starting main loop");
    ros::Rate sleep_timer(0.5);
    while (ros::ok()) {
        //have a tf of base_link w/rt map (from amcl), derived from LIDAR and map
        // also have a tf of base_link w/rt odom (from kinematics)... but this is unrealistically good
        // can run: rosrun mobot_drifty_odom mobot_drifty_odom, to get base_link w/rt drifty_odom
        //odomTf.tfListener_->lookupTransform("odom", "base_link", ros::Time(0), stfBaseLinkWrtOdom);
        odomTf.tfListener_->lookupTransform("map", "odom", ros::Time(0), stfOdomWrtMap);
        
        stfBaseLinkWrtDriftyOdom = odomTf.get_tfBaseLinkWrtDriftyOdom();
        cout<<"stfBaseLinkWrtDriftyOdom: "<<endl;
        odomTf.printStampedTf(stfBaseLinkWrtDriftyOdom);      
        
        odomTf.multiply_stamped_tfs(stfOdomWrtMap,stfBaseLinkWrtDriftyOdom,stfBaseLinkWrtMap);

        cout<<"base_link wrt map via drifty odom: "<<endl;
        odomTf.printStampedTf(stfBaseLinkWrtMap);
        sleep_timer.sleep();
        
        
    }
    
    return 0;
} 

