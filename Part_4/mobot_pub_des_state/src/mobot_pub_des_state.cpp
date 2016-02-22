#include <queue>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>
#include<mobot_pub_des_state/pathCmdAction.h>
#include <std_srvs/Trigger.h>

//constants and parameters:
const double dt = 0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
//dynamic parameters: should be tuned for target system
const double accel_max = 1.0; //1m/sec^2
const double alpha_max = 1.0; //1 rad/sec^2
const double speed_max = 1.0; //1 m/sec
const double omega_max = 1.0; //1 rad/sec
const double path_move_tol = 0.01; // if path points are within 1cm, fuggidaboutit

const int E_STOPPED = 0; //define some mode keywords
const int DONE_W_SUBGOAL = 1;
const int PURSUING_SUBGOAL = 2;
const int HALTING = 3;

class DesStatePublisher {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<mobot_pub_des_state::pathCmdAction> as_;

    // message types to communicate with our client(s)
    mobot_pub_des_state::pathCmdGoal goal_; // goal message, received from client
    mobot_pub_des_state::pathCmdResult result_; // put results here, to be sent back to the client when done w/ goal
    mobot_pub_des_state::pathCmdFeedback feedback_; // for feedback 
    //some class member variables:
    nav_msgs::Path path_;
    std::vector<nav_msgs::Odometry> des_state_vec_;
    nav_msgs::Odometry des_state_;
    nav_msgs::Odometry halt_state_;
    nav_msgs::Odometry seg_end_state_;
    nav_msgs::Odometry seg_start_state_;
    nav_msgs::Odometry current_des_state_;
    geometry_msgs::Twist halt_twist_;
    geometry_msgs::PoseStamped start_pose_;
    geometry_msgs::PoseStamped end_pose_;
    std::queue<geometry_msgs::PoseStamped> path_queue_; //a C++ "queue" object, stores vertices as Pose points in a FIFO queue
    int motion_mode_;
    bool e_stop_trigger_; //these are intended to enable e-stop via a service
    bool e_stop_reset_;
    int traj_pt_i_;
    int npts_traj_;

    // some objects to support service and publisher
    ros::ServiceServer estop_service_;
    ros::ServiceServer estop_clear_service_;
    ros::Publisher desired_state_publisher_;
    
    //a trajectory builder object; format =  package::class object
    traj_builder::traj_builder traj_builder;

    // member methods:
    void initializePublishers();
    void initializeServices();
    bool estopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
    bool clearEstopServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<mobot_pub_des_state::pathCmdAction>::GoalConstPtr& goal);
};




//fnc to populate g_path with hard-coded points;
// this is only a stand-in for what should be an action-server callback that accepts path goals

void stuff_path() {
    double x, y, phi;
    geometry_msgs::PoseStamped vertex;
    vertex.header.frame_id = "world"; //specify the frame in which poses are defined
    while (!g_path_queue.empty()) g_path_queue.pop(); //this flushes the queue; not needed now, will in future code
    // hard-coded subgoals: fill in the interesting data: (x,y) and phi = location and heading
    //vertex 1:
    x = 1.0;
    y = 2.0;
    phi = 0.123;
    vertex = xyPhi2PoseStamped(x, y, phi); // convert x,y,phi to a pose stamped; 
    g_path_queue.push(vertex);

    //vertex 2:
    x = 3.0;
    y = 4.0;
    phi = 0.567;
    vertex = xyPhi2PoseStamped(x, y, phi); // convert x,y,phi to a pose stamped; 
    g_path_queue.push(vertex);

    //vertex 3:
    x = 5.0;
    y = 6.0;
    phi = 2.345;
    vertex = xyPhi2PoseStamped(x, y, phi); // convert x,y,phi to a pose stamped; 
    g_path_queue.push(vertex);
}

//equiv to a class constructor

void do_inits() { //similar to a constructor
    stuff_path(); // put some subgoals in the path queue
    g_end_pose = g_path_queue.front(); //first point in queue will start the sequence
    g_start_pose = g_end_pose; //will get overwritten when get next vertex from queue
    g_path_queue.pop(); // remove this subgoal from the queue

    //define a halt state; zero speed and spin, and fill with viable coords
    g_halt_twist.linear.x = 0.0;
    g_halt_twist.linear.y = 0.0;
    g_halt_twist.linear.z = 0.0;
    g_halt_twist.angular.x = 0.0;
    g_halt_twist.angular.y = 0.0;
    g_halt_twist.angular.z = 0.0;

    g_halt_state.twist.twist = g_halt_twist;
    //set the halt pose to first vertex in path; better would be
    //to get actual robot state and use that
    g_halt_state.pose.pose = g_end_pose.pose;

    //odometry message uses end pose, but also includes twist info
    g_seg_end_state.pose.pose = g_end_pose.pose;
    g_seg_end_state.twist.twist = g_halt_twist;
    g_seg_start_state = g_seg_end_state; //initialize to start planning from here
    g_current_des_state = g_seg_start_state;
    g_motion_mode = DONE_W_SUBGOAL; //init in state ready to process new goal

    g_e_stop_trigger = false; //these are intended to enable e-stop via a service
    g_e_stop_reset = false;
    int g_traj_pt_i = 0;
    int g_npts_traj = 0;
}

bool process_new_subgoal() {
    if (g_path_queue.empty()) return false; // no new subgoals in queue

    //if survive to here, then queue is not empty; get and process a new subgoal;
    g_start_pose = g_end_pose; //former end pose is new start pose   
    g_end_pose = g_path_queue.front(); //front of FIFO queue is new subgoal
    g_path_queue.pop(); // remove this subgoal from the queue 

    g_seg_start_state = g_seg_end_state; //previous end state is new start state

    //details of trapezoidal profile, etc, are contained in build_trajectory fnc
    build_trajectory(g_seg_start_state, g_end_pose); // fill up new g_des_state_vec with a plan
    g_seg_end_state = g_des_state_vec.back(); //remember new end state

    g_traj_pt_i = 0; //get ready to start streaming out the plan
    g_npts_traj = g_des_state_vec.size(); // which contains this many points
    g_motion_mode = PURSUING_SUBGOAL; //since have a new plan, change mode to execute it
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle n;
    //will stream (publish) desired states to this topic with this message type:
    ros::Publisher des_state_publisher = n.advertise<nav_msgs::Odometry>("/desState", 1);
    ros::Rate looprate(1 / dt); //timer for fixed publication rate   

    // main loop; publish a desired state every iteration
    while (ros::ok()) {
        switch (g_motion_mode) {
            case E_STOPPED: //this state would have to be reset by a service
                des_state_publisher.publish(g_seg_end_state);
                //do not change g_motion_mode--only service can re-enable motion
                //need to reconsider subgoal queue and set mode to DONE_W_SUBGOAL 
                break;

            case HALTING: //e-stop service callback would set this mode
                //if need to brake from e-stop, service will have computed
                // g_des_state_vec, set indices and set motion mode;
                g_current_des_state = g_des_state_vec[g_traj_pt_i];
                des_state_publisher.publish(g_current_des_state);
                g_traj_pt_i++;
                //segue from braking to halted e-stop state;
                if (g_traj_pt_i >= g_npts_traj) { //here if completed all pts of braking traj
                    g_motion_mode = E_STOPPED; //change state to remain halted
                    g_seg_end_state = g_des_state_vec.back(); //last point of halting traj
                    // make sure it has 0 twist
                    g_seg_end_state.twist.twist = g_halt_twist;
                    g_traj_pt_i = 0; // prep for next planned trajectory
                }
                break;

            case DONE_W_SUBGOAL: //suspended, pending a new subgoal
                //see if there is another subgoal is in queue; if so, pop it off
                // process_new_subgoal will see if there is another subgoal in 
                // the queue, and if so, pop it and compute a trajectory plan to this subgoal
                // this fnc will also reset indices, and set mode to PURSUING_SUBGOAL
                // if queue is empty, this fnc returns "false"
                if (process_new_subgoal()) {
                    g_motion_mode = PURSUING_SUBGOAL; // got a new plan; change mode to pursue it
                    g_seg_end_state = g_des_state_vec.back(); // last state of traj
                    g_traj_pt_i = 0; //and get ready for next subgoal                    
                } else { //no new goal? stay halted in this mode 
                    // by simply reiterating the last state sent (should have zero vel)
                    des_state_publisher.publish(g_seg_end_state);
                }
                break;

            case PURSUING_SUBGOAL: //if have remaining pts in computed traj, send them
                //extract the i'th point of our plan:
                g_current_des_state = g_des_state_vec[g_traj_pt_i];
                des_state_publisher.publish(g_current_des_state);
                g_traj_pt_i++; // increment counter to prep for next point of plan
                //check if we have clocked out all of our planned states:
                if (g_traj_pt_i >= g_npts_traj) {
                    g_motion_mode = DONE_W_SUBGOAL; //if so, indicate we are done
                    g_seg_end_state = g_des_state_vec.back(); // last state of traj
                    g_traj_pt_i = 0; //and get ready for next subgoal
                }
                break;

            default: //this should not happen
                ROS_WARN("motion mode not recognized!");
                des_state_publisher.publish(g_halt_state);
                break;
        }

        looprate.sleep(); //sleep for defined sample period, then do loop again
    }
}

