//open_loop_nav_svc:
// exclusively to move fwd or rvrs by specified value

#include <ros/ros.h>
#include <coordinator/OpenLoopNavSvc.h>
#include <iostream>
#include <string>
#include <geometry_msgs/Twist.h>
using namespace std;
//speed params:
double MAX_SPEED=0.5; // set max speed param
double ACCEL=0.5; // translational accel, m/s^2
double T_UP_MAX = MAX_SPEED/ACCEL;
double T_DOWN_MAX= T_UP_MAX;
double RAMP_UP_DIST = 0.5*T_UP_MAX*MAX_SPEED;
double TRIANGULAR_DIST = 2.0*RAMP_UP_DIST;
double DT=0.01; //choose 100Hz update rate

ros::Publisher g_twist_commander; 
geometry_msgs::Twist g_twist;

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

bool callback(coordinator::OpenLoopNavSvcRequest& request, coordinator::OpenLoopNavSvcResponse& response)
{
    ROS_INFO("open-loop nav service callback activated");
    double move_dist = request.move_distance;
    ROS_INFO("requested move distance: %f",move_dist);
    //compute and implement a triangular or trapezoidal velocity profile:

    double sign = sgn(move_dist);
    double t_const;
    double t_up_triangular;
    double t=0.0;
    ros::Rate timer(1.0/DT);
    g_twist.linear.x=0;
    //check if dist--> triangular profile:     
    // triangular_dist = 2.0*ramp_up_dist;
    if (fabs(move_dist)<=TRIANGULAR_DIST) {
        //do triangular velocity profile 0.5*move_dist = 0.5*ACCEL*t_up_triangular^2
        t_up_triangular = sqrt(fabs(move_dist)/ACCEL);
        while (t<t_up_triangular) {
            t+= DT;
            g_twist.linear.x+= sign*ACCEL*DT;
            g_twist_commander.publish(g_twist);
            timer.sleep();
        }
        //ramp down
       while (t<2.0*t_up_triangular) {
            t+= DT;
            g_twist.linear.x-= sign*ACCEL*DT;
            g_twist_commander.publish(g_twist);
            timer.sleep();
        }
        //done--send zero vel cmd
        g_twist.linear.x=0.0;
        g_twist_commander.publish(g_twist);    
        return true;
    }
    
    else {
        //do trapezoidal velocity profile
        t_const = (fabs(move_dist) - 2.0*RAMP_UP_DIST)/MAX_SPEED;
        g_twist.linear.x=0.0;
        while (t<T_UP_MAX) {
            t+= DT;
            g_twist.linear.x+= sign*ACCEL*DT;
            g_twist_commander.publish(g_twist);
            timer.sleep();
        }
        
        while (t<T_UP_MAX+t_const) {
            t+= DT;
            g_twist.linear.x= sign*MAX_SPEED;
            g_twist_commander.publish(g_twist);
            timer.sleep();
        }        
        //ramp down
       while (t<T_UP_MAX+t_const+T_DOWN_MAX) {
            t+= DT;
            g_twist.linear.x-=sign*ACCEL*DT;
            g_twist_commander.publish(g_twist);
            timer.sleep();
        }
        g_twist.linear.x=0.0;
        g_twist_commander.publish(g_twist);        
    }    
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_loop_nav_service");
  ros::NodeHandle n;
  g_twist_commander = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  g_twist.angular.x=0;
  g_twist.angular.y=0;
  g_twist.angular.z=0;
  g_twist.linear.x=0.0;
  g_twist.linear.y=0.0;
  g_twist.linear.z=0.0;
  ros::ServiceServer service = n.advertiseService("open_loop_nav_service", callback);
  ros::spin();

  return 0;
}
