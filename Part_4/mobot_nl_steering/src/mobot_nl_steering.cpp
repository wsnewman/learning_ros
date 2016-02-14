//mobot_nl_steering.cpp:
//wsn, Feb 2016
//subscribes to topics for desired state and actual state
// invokes a nonlinear steering algorithm to command speed and spin on cmd_vel topic

// this header incorporates all the necessary #include files and defines the class "SteeringController"
#include "mobot_nl_steering.h"

//CONSTRUCTOR:  
SteeringController::SteeringController(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of SteeringController");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    
    state_psi_ = 1000.0; // put in impossible value for heading; 
    //test this value to make sure we have received a viable state message
    ROS_INFO("waiting for valid state message...");
    while (state_psi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("constructor: got a state message");    
      
    //initialize desired state;  can be changed dynamically by publishing to topic /desState
    des_state_speed_ = MAX_SPEED; //can make dynamic via des_state_rcvd.twist.twist.linear.x;
    des_state_omega_ = 0.0; //des_state_rcvd.twist.twist.angular.z;
    
    // hard code a simple path: the world x axis
    des_state_x_ = 0.0; 
    des_state_y_ = 0.0; 
    des_state_psi_ = 0.0; 
      
    // make sure the speed/spin commands are set to zero
    current_speed_des_ = 0.0;  // 
    current_omega_des_ = 0.0;    

    //initialize the twist command components, all to zero
    twist_cmd_.linear.x = 0.0;
    twist_cmd_.linear.y = 0.0;
    twist_cmd_.linear.z = 0.0;
    twist_cmd_.angular.x = 0.0;
    twist_cmd_.angular.y = 0.0;
    twist_cmd_.angular.z = 0.0;
}

//member helper function to set up subscribers;
void SteeringController::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers: gazebo state and desState");
        //subscribe to gazebo messages; ONLY works in simulation
    current_state_subscriber_ = nh_.subscribe("gazebo_mobot_pose", 1, &SteeringController::gazeboPoseCallback, this); 

    // subscribe to desired-state publications
    des_state_subscriber_ = nh_.subscribe("/desState", 1, &SteeringController::desStateCallback, this); 
}


//member helper function to set up publishers;
void SteeringController::initializePublishers()
{
    ROS_INFO("Initializing Publishers"); //: cmd_vel and cmd_vel_stamped");
    cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true); // commands the robot!
    //next three are just for debug output, plotable by rqt_plot
    heading_publisher_= nh_.advertise<std_msgs::Float32>("heading", 1, true);
    heading_cmd_publisher_ = nh_.advertise<std_msgs::Float32>("heading_cmd", 1, true);
    lat_err_publisher_ = nh_.advertise<std_msgs::Float32>("lateral_err", 1, true);
}

//receive publications from gazebo via node mobot_gazebo_state;
// this stands in for a state estimator; for a real system need to create such a node
void SteeringController::gazeboPoseCallback(const geometry_msgs::Pose& gazebo_pose) {
   state_x_ = gazebo_pose.position.x; //copy the state to member variables of this object
   state_y_ = gazebo_pose.position.y;
   state_quat_ = gazebo_pose.orientation;
   state_psi_ = convertPlanarQuat2Phi(state_quat_);
}

//use this if a desired state is being published
void SteeringController::desStateCallback(const nav_msgs::Odometry& des_state_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    des_state_speed_ = des_state_rcvd.twist.twist.linear.x;
    des_state_omega_ = des_state_rcvd.twist.twist.angular.z;
    
    des_state_x_ = des_state_rcvd.pose.pose.position.x;
    des_state_y_ = des_state_rcvd.pose.pose.position.y;
    des_state_pose_ = des_state_rcvd.pose.pose;    
    des_state_quat_ = des_state_rcvd.pose.pose.orientation;
    //Convert quaternion to simple heading
    des_state_psi_ = convertPlanarQuat2Phi(des_state_quat_);   
}

//utility fnc to compute min delta angle, accounting for periodicity
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


// HERE IS THE STEERING ALGORITHM: USE DESIRED AND ACTUAL STATE TO COMPUTE AND PUBLISH CMD_VEL
void SteeringController::mobot_nl_steering() {
    double controller_speed;
    double controller_omega;
    
    // have access to: des_state_vel_, des_state_omega_, des_state_x_, des_state_y_, 
    //  des_state_phi_ and corresponding robot state values   
    double tx = cos(des_state_psi_); // [tx,ty] is tangent of desired path
    double ty = sin(des_state_psi_); 
    double nx = -ty; //components [nx, ny] of normal to path, points to left of desired heading
    double ny = tx;   
    
    double dx = state_x_ - des_state_x_;  //x-error relative to desired path
    double dy = state_y_ - des_state_y_;  //y-error

    lateral_err_ = dx*nx+dy*ny; //lateral error is error vector dotted with path normal
                                     // lateral offset error is positive if robot is to the left of the path
    double trip_dist_err = dx*tx+dy*ty; // progress error: if positive, then we are ahead of schedule    
    //heading error: if positive, should rotate -omega to align with desired heading
    double heading_err = min_dang(state_psi_ - des_state_psi_);    
    double strategy_psi = psi_strategy(lateral_err_); //heading command, based on NL algorithm    
    controller_omega = omega_cmd_fnc(strategy_psi, state_psi_, des_state_psi_);

    controller_speed = MAX_SPEED; //default...should speed up/slow down appropriately
    
    // send out our speed/spin commands:
    twist_cmd_.linear.x = controller_speed;
    twist_cmd_.angular.z = controller_omega;
    cmd_publisher_.publish(twist_cmd_);  
        
    // DEBUG OUTPUT...
    ROS_INFO("des_state_phi, heading err = %f, %f", des_state_psi_,heading_err);
    ROS_INFO("lateral err, trip dist err = %f, %f",lateral_err_,trip_dist_err);
    std_msgs::Float32 float_msg;
    float_msg.data = lateral_err_;
    lat_err_publisher_.publish(float_msg);
    float_msg.data = state_psi_;
    heading_publisher_.publish(float_msg);
    float_msg.data = psi_cmd_;
    heading_cmd_publisher_.publish(float_msg);
    //END OF DEBUG OUTPUT   
}

double SteeringController::psi_strategy(double offset_err) {
  double psi_strategy = -(M_PI/2)*sat(offset_err/K_LAT_ERR_THRESH);
  return psi_strategy;
}

double SteeringController::omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path) {
  psi_cmd_ = psi_strategy+psi_path;
  double omega_cmd = K_PSI*(psi_cmd_ - psi_state);
  omega_cmd = MAX_OMEGA*sat(omega_cmd/MAX_OMEGA); //saturate the command at specified limit
  return omega_cmd;
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "steeringController"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type SteeringController");
    //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    SteeringController steeringController(&nh);  
 
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate, e.g. 50Hz
   
    ROS_INFO:("starting steering algorithm");
    while (ros::ok()) {
        // compute and publish twist commands 
        steeringController.mobot_nl_steering(); 
        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

