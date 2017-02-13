#include <test_plugin/test_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(test_planner::TestPlanner, nav_core::BaseLocalPlanner);

geometry_msgs::Quaternion convertPlanarPsi2Quaternion(double psi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}

double min_dphi(double phi_start,double phi_goal) {
    double dphi= phi_goal - phi_start;
    while (dphi>M_PI) dphi-=2.0*M_PI;
    while (dphi< -M_PI) dphi+=2.0*M_PI;
    return dphi;
}

// saturation function, values -1 to 1
double sat(double x) {
    if (x>1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}

double sgn(double x) {
    if (x>0.0) return 1.0;
    if (x<0.0) return -1.0;
    return 0.0;
}


test_planner::TestPlanner::TestPlanner() {
    //empty constructor; plug-in will call "initialize"
}

void test_planner::TestPlanner::initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros) {
    ros::NodeHandle nh(name);
    ROS_INFO("plug-in planner initialization: ");
    nh_ = nh;
    old_size = 0;
    tf_ = tf;  //access to a transform listener, as passed in via args
    double testval=0.0;
    //controller_rate_=20.0;
    //nh_.setParam("/move_base/controller_frequency", controller_rate_);
    controller_rate_=0;
    while (testval<0.0001) {
        ROS_INFO("reading/testing param values: ");
      //this is the frequency at which the local planner will be updated
      nh_.getParam("/move_base/controller_frequency", controller_rate_);
      ROS_INFO("controller update rate set to %f", controller_rate_);
      //here are specified dynamic limits of the mobot:
      //SHOULD use namespace of this module (TestPlanner), but here I simply
      //re-use the values specified for the default nav_stack base_local_planner,
      //as specified in base_local_planner_params.yaml in the launchfile:
      // roslaunch mobot_with_plugin mobot_startup_navstack.launch
    nh_.getParam("/move_base/TrajectoryPlannerROS/acc_lim_theta", acc_lim_theta_);
    nh_.getParam("/move_base/TrajectoryPlannerROS/acc_lim_x", acc_lim_x_);
    nh_.getParam("/move_base/TrajectoryPlannerROS/max_vel_theta", max_vel_theta_);
    nh_.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", max_vel_x_);
    nh_.getParam("/move_base/TrajectoryPlannerROS/min_vel_theta", min_vel_theta_);
    nh_.getParam("/move_base/TrajectoryPlannerROS/min_vel_x", min_vel_x_);
    ROS_INFO("acc_max = %f; alpha_max = %f",acc_lim_x_,acc_lim_theta_);
    ROS_INFO("max_vel = %f; max_omega = %f",max_vel_x_,max_vel_theta_);       
      testval=controller_rate_*acc_lim_theta_*acc_lim_x_*max_vel_theta_*max_vel_x_;
      ros::Duration(0.5).sleep();
    }
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();   
    
    omega_tol_ = 0.01;
    dist_tol_ = 0.01;
    heading_tol_ = 0.01;
    K_omega_ = 10.0; //TUNE ME!
    K_lateral_ = 3.0; //TUNE ME!
    K_travel_dist_ = 2.0; //TUNE ME!
    controller_dt_ = 1.0 / controller_rate_;
    max_reorientation_omega_ = 0.4; //slow down reorientation rotation
    nom_travel_speed_ = 0.5; //cruising speed
    des_state_vel_ = 0.0; //init planned speed
    //max brake dist: dmax = 0.5*a*t^2
    //v = a*t
    // dmax = 0.5*a*(v/a)^2 = 0.5*vmax^2/amax
    min_theta_brake_dist_= 0.5*min_vel_theta_*min_vel_theta_/acc_lim_theta_;  

    min_lin_brake_dist_ = 0.5*max_vel_x_*max_vel_x_/acc_lim_x_;
    des_decel_ = 0.2*acc_lim_x_; // more conservative than    acc_lim_x_
    des_lin_brake_dist_ = 0.5*nom_travel_speed_*nom_travel_speed_/des_decel_;
    ipose_ = 0;
    nposes_ = 0;
    //debug test: looks OK
    //ROS_INFO("enter 1 to continue: ");
    //int i;
    //std::cin>>i;
    odom_subscriber_ = nh_.subscribe("/odom", 1, &test_planner::TestPlanner::odomCallback, this); //subscribe to odom messages

    //NOTE: output topic is actually: /TestPlanner/triad_display_pose
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    test_publisher_ = nh_.advertise<std_msgs::Int32>("test_pub", 1, true); //just for test
    test_pub_val_.data = 0;
    desired_triad_pose_.pose.position.x = 0.0;
    desired_triad_pose_.pose.position.y = 0.0;
    desired_triad_pose_.pose.position.z = 0.0;
    desired_triad_pose_.pose.orientation.x = 0.0;
    desired_triad_pose_.pose.orientation.y = 0.0;
    desired_triad_pose_.pose.orientation.z = 0.0;
    desired_triad_pose_.pose.orientation.w = 1.0;
    desired_triad_pose_.header.stamp = ros::Time::now();
    desired_triad_pose_.header.frame_id = "map";            

    got_odom_=false;
    done_with_path_=true;
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
    // had problems w/ this fnc:
    //odom_phi_ = xformUtils.convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion

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
        ROS_WARN("received new plan");
    }
    path_heading_ = phiFromPoses(plan[0],plan[1]);
    tx_ = cos(path_heading_);
    ty_ = sin(path_heading_);
    nx_ = -ty_;
    ny_ = tx_;
    old_subgoal_x_= plan[0].pose.position.x;
    old_subgoal_y_ = plan[0].pose.position.y;
    new_subgoal_x_=plan[1].pose.position.x;
    new_subgoal_y_ = plan[1].pose.position.y;
    des_state_x_= old_subgoal_x_;  //initialize desired state
    des_state_y_= old_subgoal_y_;
    subgoal_dx_ = new_subgoal_x_ - old_subgoal_x_;
    subgoal_dy_ = new_subgoal_y_ - old_subgoal_y_;
    subgoal_dist_ = sqrt(subgoal_dx_*subgoal_dx_+subgoal_dy_*subgoal_dy_);   
    
    des_progress_to_subgoal_ = 0.0;
    des_state_vel_=0.0;       

    ROS_INFO("distance to first subgoal: %f",subgoal_dist_);
    //ROS_INFO("controller update rate: %f", controller_rate_);
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
    current_omega_cmd_=0;
    current_vx_cmd_=0;
    //define heading error as phi_plan - phi_robot
    heading_err_ = min_dphi(phi_base_wrt_map_,path_heading_);
    ROS_INFO("initial heading error: %f",heading_err_);
    rotating_to_start_=true; // flag that should start by rotating heading
    done_with_path_ = false;
    search_for_lethal_obs();
    return true;
}




//get tf between odom and map
//use odom info
//compute odom_wrt_map
//populate terms x_base_wrt_map_,y_base_wrt_map_,phi_base_wrt_map_
void test_planner::TestPlanner::compute_stf_base_wrt_map() {
    if (!got_odom_) return; // give up if don't have any odom values
    
    //try this fnc:
    /*
    nav_msgs::Odometry base_odom;
     odom_helper_.getOdom(base_odom);
     ROS_INFO("from odom helper: x, y = %f, %f", base_odom.position.x,base_odom.position.y);
     */
     
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
    //ROS_INFO("computed robot base pose w/rt map: ");
    //xformUtils.printStampedPose(odom_poseStamped_wrt_map_);
    x_base_wrt_map_= odom_poseStamped_wrt_map_.pose.position.x;
    y_base_wrt_map_= odom_poseStamped_wrt_map_.pose.position.y;
    geometry_msgs::Quaternion quat;
    quat = odom_poseStamped_wrt_map_.pose.orientation;
    phi_base_wrt_map_ =xformUtils.convertPlanarQuat2Phi(quat);
    ROS_INFO("mobot x,y,phi w/rt map: %f, %f, %f",x_base_wrt_map_,y_base_wrt_map_,phi_base_wrt_map_);
    //test: this agrees with: rosrun tf tf_echo map base_link 
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
    if (!got_odom_) {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;        
        return true; //do nothing until/unless have valid odom 
    }

    compute_stf_base_wrt_map(); //update robot pose w/rt map
       heading_err_ = min_dphi(phi_base_wrt_map_,path_heading_); 
       ROS_INFO("des heading, robot heading = %f, %f",path_heading_,phi_base_wrt_map_);  
        //also compute vector from previous subgoal to current x,y
        dx_from_prior_subgoal_= x_base_wrt_map_ - old_subgoal_x_;
        dy_from_prior_subgoal_= y_base_wrt_map_ - old_subgoal_y_;
        //lateral error is offset to "left" of path
        lateral_err_ = -dx_from_prior_subgoal_*ty_+dy_from_prior_subgoal_*tx_;
        projected_travel_dist_ = dx_from_prior_subgoal_*tx_ + dy_from_prior_subgoal_*ty_;
        travel_dist_err_ = des_progress_to_subgoal_ - projected_travel_dist_;
        ROS_INFO("des and actual dist from prior subgoal: %f, %f", des_progress_to_subgoal_, projected_travel_dist_);
        ROS_INFO("heading err, lateral err, travel_dist_err_: %f, %f, %f",heading_err_,lateral_err_,travel_dist_err_);         
        //if (travel_dist_err_<0) ROS_WARN("negative following distance");
        test_pub_val_.data++;
        test_publisher_.publish(test_pub_val_);
                
    //if first segment of a path, may need to reorient before proceeding  
    //rotate until phi_base_wrt_map_ matches initial path angle, 
    if(rotating_to_start_) {
        //ROS_INFO("rotating to start...");
        //use current heading, in map frame:
       //define heading error as phi_plan - phi_robot  
     
        //compute omega: this is just for spin in place
        compute_omega_rotate_to_phi(heading_err_, current_omega_cmd_);
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = current_omega_cmd_;
        //rotate to point aligned with start of plan
        //test if reoriented:
        if ((fabs(heading_err_)<heading_tol_)&&(fabs(current_omega_cmd_)<omega_tol_)) {
            rotating_to_start_=false; //reoriented; ready to start following path
            ROS_WARN("reoriented");
        }
        return true;
    } //else
    if (!done_with_path_) {
        speed_profile(); //update speed command
        done_with_path_ = update_des_state();
        //do steering feedback:
        //compute lateral offset error:
        //  cross product of two vectors:
        // old_subgoal to new_subgoal provides a direction vector = [tx,ty]
        // current (x,y)/map relative to old_subgoal--> progress vector;
        // cross product of these is offset vector; offset mag is size of this vector
        //heading_err_ = min_dphi(phi_base_wrt_map_,path_heading_); 
        //ROS_INFO("des heading, robot heading, heading err= %f, %f, %f",path_heading_,phi_base_wrt_map_
        //    ,heading_err_);
  
        current_omega_cmd_ =  K_omega_*heading_err_ - K_lateral_*lateral_err_; //+des_state_omega_
        cmd_vel.linear.x = current_vx_cmd_;
        cmd_vel.angular.z = current_omega_cmd_; 
        ROS_INFO("des x,y: %f, %f",des_state_x_,des_state_y_);
        ROS_INFO("robot x,y: %f, %f",x_base_wrt_map_,y_base_wrt_map_);
        ROS_INFO("carrot spd, commanded speed, omega:%f, %f, %f",des_state_vel_,current_vx_cmd_,current_omega_cmd_);
        return true;        
    }

    else { //if done with path, then halt--but keep servoing on orientation
        compute_omega_rotate_to_phi(heading_err_, current_omega_cmd_);
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = current_omega_cmd_;
    }
    return true;
}

double test_planner::TestPlanner::phiFromPoses(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2) {
        double x1, y1, x2, y2, dx12, dy12, ds12, phi12;
        x1 = pose1.pose.position.x;
        y1 = pose1.pose.position.y;
        x2 = pose2.pose.position.x;
        y2 = pose2.pose.position.y;
        dx12 = x2 - x1;
        dy12 = y2 - y1;
        phi12 = atan2(dy12, dx12); //heading for this segment; 
        return phi12;
}

//FIX ME!!
//given phi_des, keep sampling phi_robot;
//if fabs(phi_err)> phi_brake, ramp up omega (and check for max)
// else, decelerate w/ omega_cmd given by:
// dphi = 0.5*alpha*t^2
// omega = alpha*t
// t = omega/alpha
// dphi = 0.5*alpha*(omega/alpha)^2 = 0.5*omega^2/alpha
// omega = sqrt(2.0*dphi*alpha) (and fix sign)
// halt when omega is in range, omega_tol

void test_planner::TestPlanner::compute_omega_rotate_to_phi(double heading_err, double &omega_cmd) {
    //use: max_theta_brake_dist_, min_vel_theta_,acc_lim_theta_, K_omega_, controller_dt_
    //double approach_sign = sgn(heading_err); 
    //double approach_dist = fabs(heading_err);
    //double abs_braking_omega_cmd = sqrt(2.0*approach_dist*acc_lim_theta_);
    omega_cmd = K_omega_*heading_err;
    //could be large heading; restrict rotation rate to low value to limit overshoot
    if (fabs(omega_cmd)> max_reorientation_omega_) { //make this rotation slower
        omega_cmd = max_reorientation_omega_*sgn(omega_cmd);
    }
}

bool test_planner::TestPlanner::update_des_state() {
        //uses: speed_cmd_
        //changes: des_state_x_,des_state_y_,
        // if advance to new ipose_, then also changes:
        //  path_heading_,  tx_,ty_,nx_,ny_,
        //old_subgoal_x_, old_subgoal_y_, new_subgoal_x_, new_subgoal_y_ 
        //des_state_x_= old_subgoal_x_;  //initialize desired state
        // des_state_y_= old_subgoal_y_;
        // subgoal_dx_ = new_subgoal_x_ - old_subgoal_x_;
        // subgoal_dy_ = new_subgoal_y_ - old_subgoal_y_;
        //subgoal_dist_ = sqrt(subgoal_dx_*subgoal_dx_+subgoal_dy_*subgoal_dy_);   
        //des_progress_to_subgoal_ = 0.0;
    bool done_with_path=false;
    //march the "carrot" forward:
    des_state_x_+= des_state_vel_*controller_dt_*tx_;  //current_vx_cmd is forward speed
    des_state_y_+= des_state_vel_*controller_dt_*ty_;
    des_progress_to_subgoal_+= fabs(des_state_vel_)*controller_dt_;
    if (des_progress_to_subgoal_>subgoal_dist_) {
      done_with_path = update_path_subgoal();
    }
    return done_with_path;
}

bool test_planner::TestPlanner::update_path_subgoal() {
        geometry_msgs::PoseStamped pose1, pose2, pose3;
    //update params for next path segment
    ipose_++;
    if (ipose_>= nposes_-1) return true; // out of poses, so done with path
    //if here, then process the next subgoal:
        pose2 = g_plan_[ipose_]; //new subgoal
        pose1 = g_plan_[ipose_ - 1]; //prior subgoal
        
        
     path_heading_ = phiFromPoses(pose1,pose2);
     tx_ = cos(path_heading_);
     ty_ = sin(path_heading_);
    old_subgoal_x_= new_subgoal_x_; //plan[0].pose.position.x;
    old_subgoal_y_ = new_subgoal_y_; //plan[0].pose.position.y;
    new_subgoal_x_=pose2.pose.position.x;
    new_subgoal_y_ = pose2.pose.position.y;
    ROS_WARN("prior subgoal: x,y = %f, %f",old_subgoal_x_,old_subgoal_y_);
    ROS_WARN("new subgoal: x,y,theta = %f, %f, %f",new_subgoal_x_,new_subgoal_y_,path_heading_);
    ROS_WARN("tx,ty = %f, %f",tx_,ty_);
    //std::cout<<"pose2 frame_id = "<<pose2.header.frame_id<<std::endl;
    des_state_x_= old_subgoal_x_;  //initialize desired state
    des_state_y_= old_subgoal_y_;
    subgoal_dx_ = new_subgoal_x_ - old_subgoal_x_;
    subgoal_dy_ = new_subgoal_y_ - old_subgoal_y_;
    subgoal_dist_ = sqrt(subgoal_dx_*subgoal_dx_+subgoal_dy_*subgoal_dy_);   
    des_progress_to_subgoal_ = 0.0;        
    //display this subgoal:
    desired_triad_pose_ = pose2;
    desired_triad_pose_.header.stamp = ros::Time::now();
    desired_triad_pose_.header.frame_id = "map"; 
    desired_triad_pose_.pose.orientation = convertPlanarPsi2Quaternion(path_heading_);
    //publish the desired frame pose to be displayed as a triad marker:   
    //ROS_INFO("publishing subgoal pose...");
    pose_publisher_.publish(desired_triad_pose_);
    

    return false; //not done with path
}

void    test_planner::TestPlanner::speed_profile() {
    double lookahead_dist =
       path_lookahead_dist(des_progress_to_subgoal_,ipose_,des_lin_brake_dist_,g_plan_);
    ROS_INFO("lookahead_dist= %f",lookahead_dist);        
    //des_lin_brake_dist_ = 0.5*max_vel_x_*max_vel_x_/des_decel_;
    if (lookahead_dist < des_lin_brake_dist_) {
        //scheduled decel to halt:
 
        des_state_vel_ = sqrt(2.0*lookahead_dist*des_decel_);
        if (des_state_vel_>nom_travel_speed_) des_state_vel_=nom_travel_speed_;
        ROS_WARN("planned braking: dist %f, velnom %f",lookahead_dist,des_state_vel_);       
        current_vx_cmd_= K_travel_dist_*travel_dist_err_; //+des_state_vel_;
        return;
    }
    else if (des_state_vel_<nom_travel_speed_) {
        des_state_vel_+= acc_lim_x_*controller_dt_*0.5; //ramp up planned travel spd
        current_vx_cmd_= des_state_vel_ + K_travel_dist_*travel_dist_err_;
        return;
    }
    else 
    {
        des_state_vel_ = nom_travel_speed_; //keep carrot moving at this speed
        current_vx_cmd_= des_state_vel_ + K_travel_dist_*travel_dist_err_;
        }
}

//use this function to compute the distance-to-go.
//however, don't bother looking forward for distance> max_search_dist
double test_planner::TestPlanner::path_lookahead_dist(double dist_progress_to_node_i,int node_i,double max_search_dist,
        std::vector<geometry_msgs::PoseStamped> plan) {
     geometry_msgs::PoseStamped pose1, pose2;
     int n_nodes = plan.size();
     if (node_i>= n_nodes) {
         ROS_WARN("index exceeds number of poses in plan!!");
         return 0.0; // illegal; just return 0
     }
     if (node_i<1) {
         ROS_WARN("warning: node_i = %d; resetting to 1",node_i);
     }

        //for first node pair, account for progress from pose1 to pose2
        double lookahead_dist = -dist_progress_to_node_i;
        while ((lookahead_dist<max_search_dist)&&(node_i<=n_nodes-1)) {
            //add up incremental distances btwn nodes
            pose2 = plan[node_i]; //new subgoal
            pose1 = plan[node_i - 1]; //prior subgoal            
            lookahead_dist+= dist_btwn_poses(pose1,pose2);
            node_i++;
        }
        return lookahead_dist;
}

double test_planner::TestPlanner::dist_btwn_poses(geometry_msgs::PoseStamped pose1,
        geometry_msgs::PoseStamped pose2) {
    double segment_dx,segment_dy,segment_length;
    segment_dx = pose2.pose.position.x-pose1.pose.position.x;
    segment_dy = pose2.pose.position.y-pose1.pose.position.y;
    segment_length = sqrt(segment_dx*segment_dx+segment_dy*segment_dy);
    return segment_length;
    
}

//this fnc illustrates how to access values from within a cost map
//converts map coords (in meters) into map indices, then looks up
//the cost value in corresponding cell
void test_planner::TestPlanner::search_for_lethal_obs() {
    //x_base_wrt_map_,y_base_wrt_map_
    unsigned int cell_x, cell_y;
    unsigned char cost;
    int int_cost;
    double x = x_base_wrt_map_;
    double y = y_base_wrt_map_;
    for (x = x_base_wrt_map_;x<x_base_wrt_map_+1.0;x+=0.02) {
       costmap_->worldToMap(x, y, cell_x, cell_y);
       cost = costmap_->getCost(cell_x, cell_y);   
       int_cost = (int) cost;
       ROS_INFO("cost(%f,%f) = %d",x,y,int_cost);
    }
}