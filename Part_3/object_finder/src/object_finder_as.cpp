// action server to respond to perception requests
// Wyatt Newman

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<pcl_utils/pcl_utils.h>
#include<object_finder/objectFinderAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <xform_utils/xform_utils.h>

Eigen::Affine3f g_affine_kinect_wrt_base;

class ObjectFinder {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<object_finder::objectFinderAction> object_finder_as_;

    // here are some message types to communicate with our client(s)
    object_finder::objectFinderGoal goal_; // goal message, received from client
    object_finder::objectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    object_finder::objectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    PclUtils pclUtils_;
    tf::TransformListener* tfListener_;

    //specialized function to find an upright Coke can on known height of horizontal surface;
    // returns true/false for found/not-found, and if found, fills in the object pose
    bool find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose);
    bool find_toy_block(float surface_height, geometry_msgs::PoseStamped &object_pose);
    float find_table_height();
    double surface_height_;
    bool found_surface_height_;
public:
    ObjectFinder(); //define the body of the constructor outside of class definition

    ~ObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal);
    XformUtils xformUtils_;
};

ObjectFinder::ObjectFinder() :
object_finder_as_(nh_, "object_finder_action_service", boost::bind(&ObjectFinder::executeCB, this, _1), false), pclUtils_(&nh_) {
    ROS_INFO("in constructor of ObjectFinder...");
    // do any other desired initializations here...specific to your implementation
 
    object_finder_as_.start(); //start the server running
    tfListener_ = new tf::TransformListener; //create a transform listener
    found_surface_height_=false;
    
}

//specialized function: DUMMY...JUST RETURN A HARD-CODED POSE; FIX THIS

bool ObjectFinder::find_upright_coke_can(float surface_height, geometry_msgs::PoseStamped &object_pose) {
    bool found_object = true;
    object_pose.header.frame_id = "world";
    object_pose.pose.position.x = 0.680;
    object_pose.pose.position.y = -0.205;
    object_pose.pose.position.z = surface_height;
    object_pose.pose.orientation.x = 0;
    object_pose.pose.orientation.y = 0;
    object_pose.pose.orientation.z = 0;
    object_pose.pose.orientation.w = 1;
    return found_object;

}

bool ObjectFinder::find_toy_block(float surface_height, geometry_msgs::PoseStamped &object_pose) {
    Eigen::Vector3f plane_normal;
    double plane_dist;
    //bool valid_plane;
    Eigen::Vector3f major_axis;
    Eigen::Vector3f centroid;
    bool found_object = true; //should verify this
    double block_height = 0.035; //this height is specific to the TOY_BLOCK model
    //if insufficient points in plane, find_plane_fit returns "false"
    //should do more sanity testing on found_object status
    //hard-coded search bounds based on a block of width 0.035
    found_object = pclUtils_.find_plane_fit(0.4, 1, -0.5, 0.5, surface_height + 0.025, surface_height + 0.045, 0.001,
            plane_normal, plane_dist, major_axis, centroid);
    // need more here, if want to distinguish different types of blocks;
    // write a new pclUtils fnc for this. Then set:
    //result_.object_id = CODE_OF_BLOCK_FOUND;
    //
    if (plane_normal(2) < 0) plane_normal(2) *= -1.0; //in world frame, normal must point UP
    Eigen::Matrix3f R;
    Eigen::Vector3f y_vec;
    R.col(0) = major_axis;
    R.col(2) = plane_normal;
    R.col(1) = plane_normal.cross(major_axis);
    Eigen::Quaternionf quat(R);
    object_pose.header.frame_id = "base_link";
    object_pose.pose.position.x = centroid(0);
    object_pose.pose.position.y = centroid(1);
    //the TOY_BLOCK model has its origin in the middle of the block, not the top surface
    //so lower the block model origin by half the block height from upper surface
    object_pose.pose.position.z = centroid(2)-0.5*block_height;
    //create R from normal and major axis, then convert R to quaternion

    object_pose.pose.orientation.x = quat.x();
    object_pose.pose.orientation.y = quat.y();
    object_pose.pose.orientation.z = quat.z();
    object_pose.pose.orientation.w = quat.w();
    return found_object;
}

float ObjectFinder::find_table_height() {
    int npts_plane_max = 0;
    int npts_slab;
    double z_eps = 0.005;
    double table_height = 0.0;
    vector<int> indices;
    for (double plane_height = 0.6; plane_height < 1.2; plane_height += z_eps) {
        pclUtils_.find_coplanar_pts_z_height(plane_height, z_eps, indices);
        npts_slab = (int) indices.size();
        ROS_INFO("height %f has npts  = %d", plane_height, npts_slab);
        if (npts_slab > npts_plane_max) {
            npts_plane_max = npts_slab;
            table_height = plane_height;
        }
    }
    return (table_height);
}

//specified surface height meaning is height of surface of table top

void ObjectFinder::executeCB(const actionlib::SimpleActionServer<object_finder::objectFinderAction>::GoalConstPtr& goal) {
    int object_id = goal->object_id;
    geometry_msgs::PoseStamped object_pose;
    bool known_surface_ht = goal->known_surface_ht;
    float surface_height;
    if (known_surface_ht) {
        surface_height = goal->surface_ht;
    }
    bool found_object = false;
    //get a fresh snapshot:
    pclUtils_.reset_got_kinect_cloud();
    while (!pclUtils_.got_kinect_cloud()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        ros::Duration(0.1).sleep();
        ROS_INFO("waiting for snapshot...");
    }
    
    //if here, have a new cloud in *pclKinect_ptr_; transform this cloud to base-frame coords:
    ROS_INFO("transforming point cloud");
    pclUtils_.transform_kinect_cloud(g_affine_kinect_wrt_base);
    // find tabletop...try different methods and time them
    if (!known_surface_ht) {
        ros::Time tstart = ros::Time::now();
        double table_ht;
        //hard-coded search range: x= [0,1], y= [-0.5,0.5], z=[0.6,1.2] in steps of 0.005
        table_ht = pclUtils_.find_table_height(0.0, 1, -0.5, 0.5, 0.6, 1.2, 0.005);
        ROS_INFO("table ht: %f", table_ht);
        ros::Time t3 = ros::Time::now();
        surface_height_ = table_ht; //remember this value for potential future use
        found_surface_height_ = true;
    }
    result_.object_id = goal->object_id; //by default, set the "found" object_id to the "requested" object_id
    //note--finder might change this ID, if warranted

    switch (object_id) {
        case ObjectIdCodes::COKE_CAN_UPRIGHT:
            //specialized function to find an upright Coke can on a horizontal surface of known height:
            found_object = find_upright_coke_can(surface_height, object_pose); //special case for Coke can;
            if (found_object) {
                ROS_INFO("found upright Coke can!");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_pose = object_pose;
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                object_finder_as_.setAborted(result_);
            }
            break;
        case ObjectIdCodes::TOY_BLOCK_ID:
            //specialized function to find toy block model
            found_object = find_toy_block(surface_height, object_pose); //special case for toy block
            if (found_object) {
                ROS_INFO("found toy block!");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                result_.object_pose = object_pose;
                object_finder_as_.setSucceeded(result_);
            } else {
                ROS_WARN("could not find requested object");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_NOT_FOUND;
                object_finder_as_.setAborted(result_);
            }
            break;
        case ObjectIdCodes::TABLE_SURFACE:
            //if called with !known_surface_ht, then surface height was found above;
            // return the computed surface_height within an object_pose
            ROS_INFO("object finder: finding/returning table height");
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                //rtn result in an object_pose: surface height w/rt world
                object_pose.header.frame_id = "base_link";
                object_pose.pose.position.x = 0.5; //arbitrarily place origin 0.5m in front of robot
                object_pose.pose.position.y = 0.0; //centered, left/right
                object_pose.pose.position.z = surface_height_; //computed value w/rt base_link
                object_pose.pose.orientation.x = 0; //and aligned with base frame orientation
                object_pose.pose.orientation.y = 0;
                object_pose.pose.orientation.z = 0;
                object_pose.pose.orientation.w = 1;
                result_.object_pose = object_pose;
                ROS_INFO("returning height %f",surface_height_);
                result_.found_object_code = object_finder::objectFinderResult::OBJECT_FOUND;
                object_finder_as_.setSucceeded(result_);             
            break;
        default:
            ROS_WARN("this object ID is not implemented");
            result_.found_object_code = object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED;
            object_finder_as_.setAborted(result_);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_finder_node"); // name this node 

    ROS_INFO("instantiating the object finder action server: ");

    ObjectFinder object_finder_as; // create an instance of the class "ObjectFinder"
    tf::TransformListener tfListener;
    ROS_INFO("listening for kinect-to-base transform:");
    tf::StampedTransform stf_kinect_wrt_base;
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and base_link...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("base_link", "kinect_pc_frame", ros::Time(0), stf_kinect_wrt_base);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("kinect to base_link tf is good");
    object_finder_as.xformUtils_.printStampedTf(stf_kinect_wrt_base);
    tf::Transform tf_kinect_wrt_base = object_finder_as.xformUtils_.get_tf_from_stamped_tf(stf_kinect_wrt_base);
    g_affine_kinect_wrt_base = object_finder_as.xformUtils_.transformTFToAffine3f(tf_kinect_wrt_base);
    cout << "affine rotation: " << endl;
    cout << g_affine_kinect_wrt_base.linear() << endl;
    cout << "affine offset: " << g_affine_kinect_wrt_base.translation().transpose() << endl;

    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        ros::Duration(0.1).sleep();
    }

    return 0;
}

