#ifndef TEST_PLANNER_CPP
#define TEST_PLANNER_CPP

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <geometry_msgs/Quaternion.h>
using namespace std;

namespace test_planner {
	class TestPlanner : public nav_core::BaseLocalPlanner {
	public:
		TestPlanner();
		//TestPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/** overridden classes from interface nav_core::BaseGlobalPlanner **/
		void initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros);
		bool isGoalReached();
		bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);
		bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
		void printPose(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2);
                void odomCallback(const nav_msgs::Odometry& odom_rcvd);
                XformUtils xformUtils; //instantiate an object of XformUtils
                void compute_stf_base_wrt_map();
	private:
		ros::Time tg;
		unsigned int old_size;
		tf::TransformListener * handed_tf;
                ros::Subscriber odom_subscriber_;
                std::vector< geometry_msgs::PoseStamped > g_plan_;
                ros::NodeHandle nh_;
                double controller_rate_, controller_dt_;
                int ipose_,nposes_;
                tf::StampedTransform stfBaseLinkWrtOdom_; //base link w/rt odom frame; get this from tf; 
                tf::StampedTransform stfOdomWrtMap_;
                tf::StampedTransform stfEstBaseWrtMap_; 
                tf::TransformListener * tf_;
                nav_msgs::Odometry current_odom_;
                geometry_msgs::Pose odom_pose_;
                double odom_x_,odom_y_;
                double odom_phi_;
                double x_base_wrt_map_,y_base_wrt_map_,phi_base_wrt_map_;
                geometry_msgs::Quaternion odom_quat_; 
                bool got_odom_;
                geometry_msgs::PoseStamped odom_poseStamped_wrt_map_;
	};	
};
#endif
