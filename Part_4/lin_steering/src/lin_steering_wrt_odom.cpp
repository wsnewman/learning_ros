//lin_steering_wrt_odom.cpp:
//wsn, Feb 2016
//subscribe to desired state and to odom
// invoke an algorithm to command speed and spin
// update this computation adequately fast



// this header incorporates all the necessary #include files and defines the class "SteeringController"
#include "steering_algorithm.h"


SteeringController::SteeringController(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of SteeringController");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    //initializeServices();
    
    odom_phi_ = 1000.0; // put in impossible value for heading; test this value to make sure we have received a viable odom message
    ROS_INFO("waiting for valid odom message...");
    while (odom_phi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("constructor: got an odom message");    
    
    /*
    tfListener_ = new tf::TransformListener; 
 
    bool tferr=true;
    ROS_INFO("waiting for tf...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener_->lookupTransform("odom", "map", ros::Time(0), mapToOdom_);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    // from now on, tfListener will keep track of transforms from map frame to target frame
    */
    
    //initialize desired state, in case this is not yet being published adequately
    des_state_ = current_odom_;  // use the current odom state
    // but make sure the speed/spin commands are set to zero
    current_speed_des_ = 0.0;  // 
    current_omega_des_ = 0.0;    
    des_state_.twist.twist.linear.x = current_speed_des_; // but specified desired twist = 0.0
    des_state_.twist.twist.angular.z = current_omega_des_;
    des_state_.header.stamp = ros::Time::now();   

    //initialize the twist command components, all to zero
    twist_cmd_.linear.x = 0.0;
    twist_cmd_.linear.y = 0.0;
    twist_cmd_.linear.z = 0.0;
    twist_cmd_.angular.x = 0.0;
    twist_cmd_.angular.y = 0.0;
    twist_cmd_.angular.z = 0.0;

    twist_cmd2_.twist = twist_cmd_; // copy the twist command into twist2 message
    twist_cmd2_.header.stamp = ros::Time::now(); // look up the time and put it in the header  

}

//member helper function to set up subscribers;
void SteeringController::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers: odom and desState");
    odom_subscriber_ = nh_.subscribe("/odom", 1, &SteeringController::odomCallback, this); //subscribe to odom messages
    // add more subscribers here, as needed
    des_state_subscriber_ = nh_.subscribe("/desState", 1, &SteeringController::desStateCallback, this); // for desired state messages
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
/*
void SteeringController::initializeServices()
{
    ROS_INFO("Initializing Services: exampleMinimalService");
    simple_service_ = nh_.advertiseService("exampleMinimalService",
                                                   &SteeringController::serviceCallback,
                                                   this);  
    // add more services here, as needed
}
*/

//member helper function to set up publishers;
void SteeringController::initializePublishers()
{
    ROS_INFO("Initializing Publishers: cmd_vel and cmd_vel_stamped");
    cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true); // talks to the robot!
    cmd_publisher2_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1, true); //alt topic, includes time stamp
    //steering_errs_publisher_ =  nh_.advertise<std_msgs::Float32MultiArray>("steering_errs",1, true);
}



void SteeringController::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    current_odom_ = odom_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    odom_pose_ = odom_rcvd.pose.pose;
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    odom_quat_ = odom_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    odom_phi_ = convertPlanarQuat2Phi(odom_quat_); // cheap conversion from quaternion to heading for planar motion
    // let's put odom x,y in an Eigen-style 2x1 vector; convenient for linear algebra operations
    //odom_xy_vec_(0) = odom_x_;
    //odom_xy_vec_(1) = odom_y_;   
}

void SteeringController::desStateCallback(const nav_msgs::Odometry& des_state_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    des_state_ = des_state_rcvd; // save the entire message
    // but also pick apart pieces, for ease of use
    des_state_pose_ = des_state_rcvd.pose.pose;
    des_state_vel_ = des_state_rcvd.twist.twist.linear.x;
    des_state_omega_ = des_state_rcvd.twist.twist.angular.z;
    des_state_x_ = des_state_rcvd.pose.pose.position.x;
    des_state_y_ = des_state_rcvd.pose.pose.position.y;
    des_state_quat_ = des_state_rcvd.pose.pose.orientation;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    des_state_phi_ = convertPlanarQuat2Phi(des_state_quat_); // cheap conversion from quaternion to heading for planar motion
    // fill in an Eigen-style 2x1 vector as well--potentially convenient for linear algebra operations    
    //des_xy_vec_(0) = des_state_x_;
    //des_xy_vec_(1) = des_state_y_;      
}

//utility fnc to compute min dang, accounting for periodicity
double SteeringController::min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}


// saturation function, values -1 to 1
double SteeringController::sat(double x) {
    if (x>1.0) {
        return 1.0;
    }
    if (x< -1.0) {
        return -1.0;
    }
    return x;
}

//some conversion utilities:
double SteeringController::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

/*
//member function implementation for a service callback function
// could do something useful with this
bool SteeringController::serviceCallback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    return true;
}
*/

// HERE IS THE BIG DEAL: USE DESIRED AND ACTUAL STATE TO COMPUTE AND PUBLISH CMD_VEL
void SteeringController::lin_steering_algorithm() {
    double controller_speed;
    double controller_omega;
    //Eigen::Vector2d pos_err_xy_vec_;
    //Eigen::Vector2d t_vec;    //tangent of desired path
    //Eigen::Vector2d n_vec;    //normal to desired path, pointing to the "left" 
    double tx = cos(des_state_phi_);
    double ty = sin(des_state_phi_);
    double nx = -ty;
    double ny = tx;
    
    double heading_err;  
    double lateral_err;
    double trip_dist_err; // error is scheduling...are we ahead or behind?
    

    // have access to: des_state_vel_, des_state_omega_, des_state_x_, des_state_y_, des_state_phi_ and corresponding odom values    
    double dx = des_state_x_- odom_x_;
    double dy = des_state_y_ - odom_y_;
    
    //pos_err_xy_vec_ = des_xy_vec_ - odom_xy_vec_; // vector pointing from odom x-y to desired x-y
    //lateral_err = n_vec.dot(pos_err_xy_vec_); //signed scalar lateral offset error; if positive, then desired state is to the left of odom
    lateral_err = dx*nx + dy*ny;
    trip_dist_err = dx*tx + dy*ty;
    
    //trip_dist_err = t_vec.dot(pos_err_xy_vec_); // progress error: if positive, then we are behind schedule
    heading_err = min_dang(des_state_phi_ - odom_phi_); // if positive, should rotate +omega to align with desired heading
    
    
    // DEBUG OUTPUT...
    ROS_INFO("des_state_phi, odom_phi, heading err = %f, %f, %f", des_state_phi_,odom_phi_,heading_err);
    ROS_INFO("lateral err, trip dist err = %f, %f",lateral_err,trip_dist_err);
    // DEFINITELY COMMENT OUT ALL cout<< OPERATIONS FOR REAL-TIME CODE
    //std::cout<<des_xy_vec_<<std::endl;
    //std::cout<<odom_xy_vec_<<std::endl;
    // let's put these in a message to publish, for rqt_plot to display
    //steering_errs_.data.clear();
    //steering_errs_.data.push_back(lateral_err);
    //steering_errs_.data.push_back(heading_err); 
    //steering_errs_.data.push_back(trip_dist_err);

    //steering_errs_publisher_.publish(steering_errs_); // suitable for plotting w/ rqt_plot
    //END OF DEBUG STUFF
    
     // do something clever with this information     
    controller_speed = des_state_vel_ + K_TRIP_DIST*trip_dist_err; //speed up/slow down to null out 
    //controller_omega = des_state_omega_; //ditto
    controller_omega = des_state_omega_ + K_PHI*heading_err + K_DISP*lateral_err;
    
    controller_omega = MAX_OMEGA*sat(controller_omega/MAX_OMEGA); // saturate omega command at specified limits
    
    // send out our very clever speed/spin commands:
    twist_cmd_.linear.x = controller_speed;
    twist_cmd_.angular.z = controller_omega;
    twist_cmd2_.twist = twist_cmd_; // copy the twist command into twist2 message
    twist_cmd2_.header.stamp = ros::Time::now(); // look up the time and put it in the header 
    cmd_publisher_.publish(twist_cmd_);  
    cmd_publisher2_.publish(twist_cmd2_);     
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "steeringController"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type SteeringController");
    SteeringController steeringController(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate, e.g. 50Hz
   
    ROS_INFO:("starting steering algorithm");
    while (ros::ok()) {
        steeringController.lin_steering_algorithm(); // compute and publish twist commands and cmd_vel and cmd_vel_stamped

        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

