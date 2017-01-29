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
    handed_tf = tf;
    if (nh_.getParam("/move_base/controller_frequency", controller_rate_)) {
        ROS_INFO("controller update rate set to %f", controller_rate_);
        if (controller_rate_ < 100.0) {
            controller_dt_ = 1.0 / controller_rate_;
        } else {
            controller_dt_ = 1.0 / 100.0;
        }
    } else {
        ROS_WARN("could not find parameter value /move_base/controller_frequency");
    }
    ipose_ = 0;
    nposes_ = 0;
    //debug test: looks OK
    //ROS_INFO("enter 1 to continue: ");
    //int i;
    //std::cin>>i;

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
    ROS_INFO("poses of plan: ");
    int nposes = plan.size();
    geometry_msgs::PoseStamped ipose1, ipose2;
    for (int i = 1; i < nposes; i++) {
        ipose1 = plan[i - 1];
        ipose2 = plan[i];
        printPose(ipose1, ipose2);
    }
    //If we wait long enough, the global planner will ask us to follow the same plan again. This would reset the five-
    //second timer if I just had it refresh every time this function was called, so I check to see if the new plan is
    //"the same" as the old one first.
    if (plan.size() != old_size) {
        old_size = plan.size();
        tg = ros::Time::now() + ros::Duration(5.0);
        g_plan_ = plan;
        ipose_ = 1;
        nposes_ = plan.size();
    }

    ROS_INFO("controller update rate: %f", controller_rate_);
    return true;
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

    if (ipose_ < nposes_) {
        pose2 = g_plan_[ipose_];
        pose1 = g_plan_[ipose_ - 1];
        x1 = pose1.pose.position.x;
        y2 = pose1.pose.position.y;
        x2 = pose2.pose.position.x;
        y2 = pose2.pose.position.y;
        dx12 = x2 - x1;
        dy12 = y2 - y1;
        ds12 = sqrt(dx12 * dx12 + dy12 * dy12); //incremental distance between these sequential planned poses
        speed = ds12 / controller_dt_;
        phi12 = 2.0 * atan2(dy12, dx12); //heading for this segment;    
        cmd_vel.linear.x = speed;

        cmd_vel.angular.z = 0.0;
        if (ipose_ + 1 < nposes_) { //can we look a two points ahead?
            pose2 = g_plan_[ipose_ + 1];
            x3 = pose3.pose.position.x;
            y3 = pose3.pose.position.y;
            dx23 = x3 - x2;
            dy23 = y3 - y2;
            ds23 = sqrt(dx23 * dx23 + dy23 * dy23); //incremental distance between these sequential planned poses
            phi23 = 2.0 * atan2(dy23, dx23); //heading for this segment;    
            dphi = phi23 - phi12;
            while (dphi > M_PI) dphi -= 2.0 * M_PI;
            while (dphi< -M_PI) dphi += 2.0 * M_PI;
            omega = dphi / controller_dt_;
            cmd_vel.angular.z = omega;
        }
        ipose_++;
    } else {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }
    return true;
}
