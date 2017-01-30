#include <test_plugin/test_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(test_planner::TestPlanner, nav_core::BaseLocalPlanner);

test_planner::TestPlanner::TestPlanner() {
    //I don't think we need anything here.
}

void test_planner::TestPlanner::initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros) {
    ros::NodeHandle nh(name);
    ROS_INFO("test-planner initialization: ");
    nh_ = nh;
    old_size = 0;
    tf_ = tf;
    controller_rate_=20.0;
    nh_.setParam("/move_base/controller_frequency", controller_rate_);
    controller_rate_=0;
    while (controller_rate_<1.0) {
      nh_.getParam("/move_base/controller_frequency", controller_rate_);
      ROS_INFO("controller update rate set to %f", controller_rate_);
      ros::Duration(0.5).sleep();
    }
    
    controller_dt_ = 1.0 / controller_rate_;

    ipose_ = 0;
    nposes_ = 0;
    //debug test: looks OK
    //ROS_INFO("enter 1 to continue: ");
    //int i;
    //std::cin>>i;
    odom_subscriber_ = nh_.subscribe("/odom", 1, &test_planner::TestPlanner::odomCallback, this); //subscribe to odom messages
    got_odom_=false;

}

void test_planner::TestPlanner::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    odom_pose_ = odom_rcvd.pose.pose;
    //SHOULD transform these velocity values to map frame as well
    //odom_vel_ = odom_rcvd.twist.twist.linear.x;
    //odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    odom_phi_ = xformUtils.convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion

    got_odom_=true;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    //ROS_INFO("odom x,y = %f, %f",odom_x_,odom_y_);
}
    
bool test_planner::TestPlanner::isGoalReached() {
    //For demonstration purposes, sending a sing_plan_avpoint will cause five seconds of activity before exiting.
    //return ros::Time::now() > tg;
    return (ipose_ >= nposes_);
}

bool test_planner::TestPlanner::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan) {
    //The "plan" that comes in here is a bunch of poses of variable length, calculated by the global planner.
    //We're just ignoring it entirely, but an actual planner would probably take this opportunity
    //to store it somewhere and maybe update components that reference it.
    ROS_INFO("GOT A PLAN OF SIZE %lu", plan.size());

    int nposes = plan.size();
    /*
    ROS_INFO("poses of plan: ");
    geometry_msgs::PoseStamped ipose1, ipose2;
    for (int i = 1; i < nposes; i++) {
        ipose1 = plan[i - 1];
        ipose2 = plan[i];
        printPose(ipose1, ipose2);
    }
     * */
    //If we wait long enough, the global planner will ask us to follow the same plan again. This would reset the five-
    //second timer if I just had it refresh every time this function was called, so I check to see if the new plan is
    //"the same" as the old one first.
    // FIX THIS
    if (plan.size() != old_size) {
        old_size = plan.size();
        tg = ros::Time::now() + ros::Duration(5.0);
        g_plan_ = plan;
        ipose_ = 1;
        nposes_ = plan.size();
    }

    ROS_INFO("controller update rate: %f", controller_rate_);
        bool tferr = true;
    ROS_INFO("waiting for tf between base_link and odom...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform from target frame "odom" to source frame "link2"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tf_->lookupTransform("odom", "base_link", ros::Time(0), stfBaseLinkWrtOdom_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("odom to base_link tf is good");
    
    //now check for xform base_link w/rt map; would need amcl, or equivalent running
    tferr = true;
    ROS_INFO("waiting for tf between odom and map...");
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform from target frame "odom" to source frame "link2"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. 
            //See tf/CoordinateFrameConventions#Transform_Direction
            tf_->lookupTransform("map", "odom", ros::Time(0), stfOdomWrtMap_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("map to odom tf is good");
    // get robot x,y,phi from odom and stfOdomWrtMap_
    // convert odom_pose_ into a stf, then do multiply, then pull out terms and convert phi
    
    compute_stf_base_wrt_map();
    return true;
}

//get tf between odom and map
//use odom info
//compute odom_wrt_map
//populate terms x_base_wrt_map_,y_base_wrt_map_,phi_base_wrt_map_
void test_planner::TestPlanner::compute_stf_base_wrt_map() {
    if (!got_odom_) return; // give up if don't have any odom values
    //update to latest stfOdomWrtMap
    tf_->lookupTransform("map", "odom", ros::Time(0), stfOdomWrtMap_);
    //make an stf from odom:
    //get the position from odom, converted to a tf type
    tf::Vector3 pos;
    pos.setX(odom_pose_.position.x);
    pos.setY(odom_pose_.position.y);
    pos.setZ(odom_pose_.position.z);
    stfBaseLinkWrtOdom_.stamp_ = ros::Time::now();
    stfBaseLinkWrtOdom_.setOrigin(pos);
    // also need to get the orientation of odom_pose and use setRotation
    tf::Quaternion q;
    q.setX(odom_quat_.x);
    q.setY(odom_quat_.y);
    q.setZ(odom_quat_.z);
    q.setW(odom_quat_.w);
    stfBaseLinkWrtOdom_.setRotation(q);
    stfBaseLinkWrtOdom_.frame_id_ = "odom";
    stfBaseLinkWrtOdom_.child_frame_id_ = "base_link";   

    xformUtils.multiply_stamped_tfs(stfOdomWrtMap_,stfBaseLinkWrtOdom_,stfEstBaseWrtMap_);  
    odom_poseStamped_wrt_map_ = xformUtils.get_pose_from_stamped_tf(stfEstBaseWrtMap_);
    ROS_INFO("computed robot base pose w/rt map: ");
    xformUtils.printStampedPose(odom_poseStamped_wrt_map_);
    x_base_wrt_map_= odom_poseStamped_wrt_map_.pose.position.x;
    y_base_wrt_map_= odom_poseStamped_wrt_map_.pose.position.y;
    geometry_msgs::Quaternion quat;
    quat = odom_poseStamped_wrt_map_.pose.orientation;
    phi_base_wrt_map_ =xformUtils.convertPlanarQuat2Phi(quat);
    ROS_INFO("mobot x,y,phi w/rt map: %f, %f, %f",x_base_wrt_map_,y_base_wrt_map_,phi_base_wrt_map_);
}

void test_planner::TestPlanner::printPose(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2) {
    double x, y, x2, y2, x3, y3, yaw, dx, dy, ds, phi, dx23, dy23, phi23;
    //geometry_msgs::Quaternion quat;
    x = pose1.pose.position.x;
    y = pose1.pose.position.y;
    x2 = pose2.pose.position.x;
    y2 = pose2.pose.position.y;
    dx = x2 - x;
    dy = y2 - y;
    ds = sqrt(dx * dx + dy * dy); //incremental distance between these sequential planned poses
    phi = 2.0 * atan2(dy, dx); //heading for this segment; //looks like global plan does NOT have orientation


    ROS_INFO("x,y,dx,dy,ds,phi = %f, %f, %f, %f, %f, %f", x2, y2, dx, dy, ds, phi);
}

bool test_planner::TestPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    //This is the meat-and-potatoes of the plugin, where velocities are actually generated.
    //SPOILER ALERT: I just set a bunch of stuff to 1.
    geometry_msgs::PoseStamped pose1, pose2, pose3;
    double x1, y1, x2, y2, dx12, dy12, ds12, phi12, x3, y3, dx23, dy23, ds23, phi23, dphi, speed, omega;
    if (!got_odom_) return true;
    
    if (ipose_ < nposes_) {
        pose2 = g_plan_[ipose_];
        pose1 = g_plan_[ipose_ - 1];
        x1 = pose1.pose.position.x;
        y1 = pose1.pose.position.y;
        x2 = pose2.pose.position.x;
        y2 = pose2.pose.position.y;
        dx12 = x2 - x1;
        dy12 = y2 - y1;
        ds12 = sqrt(dx12 * dx12 + dy12 * dy12); //incremental distance between these sequential planned poses
        speed = ds12 / controller_dt_;
        phi12 = atan2(dy12, dx12); //heading for this segment;    
        ROS_INFO("x1,y1,dx12,dy12,ds12,phi12 = %f, %f, %f, %f, %f, %f", x1, y1, dx12, dy12, ds12, phi12);
        cmd_vel.linear.x = speed;
        ROS_INFO("speed: %f",speed);

        cmd_vel.angular.z = 0.0;
        if (ipose_ + 1 < nposes_) { //can we look a two points ahead?
            pose3 = g_plan_[ipose_ + 1];
            x3 = pose3.pose.position.x;
            y3 = pose3.pose.position.y;
            dx23 = x3 - x2;
            dy23 = y3 - y2;
            ds23 = sqrt(dx23 * dx23 + dy23 * dy23); //incremental distance between these sequential planned poses
            phi23 = atan2(dy23, dx23); //heading for this segment;   
            ROS_INFO("x3,y3,dx23,dy23,ds23,phi23 = %f, %f, %f, %f, %f, %f",x3,y3,dx23,dy23,ds23,phi23);
            
            dphi = phi23 - phi12;
            while (dphi > M_PI) dphi -= 2.0 * M_PI;
            while (dphi< -M_PI) dphi += 2.0 * M_PI;
            
            omega = dphi / controller_dt_;
            ROS_INFO("phi12, phi23, dphi, omega = %f, %f, %f, %f",phi12,phi23,dphi,omega);
            cmd_vel.angular.z = omega;
        }
        ipose_++;
    } else {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }
    return true;
}
