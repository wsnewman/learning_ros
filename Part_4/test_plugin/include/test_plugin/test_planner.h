#ifndef TEST_PLANNER_CPP
#define TEST_PLANNER_CPP

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>

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
                double phiFromPoses(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2);
                bool compute_omega_rotate_to_start(double &omega_cmd);
                //input heading err and current omega (presumed = omega_cmd);
                //fnc updates appropriate omega_cmd to impose over next control period
                //calling function is responsible for updating measurement of heading_err
                void compute_omega_rotate_to_phi(double heading_err, double &omega_cmd);
                bool update_des_state();
                bool update_path_subgoal();
                void    speed_profile();
                //search forward thru nodes of path plan to see if need to start braking
                double path_lookahead_dist(double dist_progress_to_node_i,int node_i,double max_search_dist,
        std::vector<geometry_msgs::PoseStamped> plan);
                double dist_btwn_poses(geometry_msgs::PoseStamped pose1,
        geometry_msgs::PoseStamped pose2);
                void search_for_lethal_obs();
	private:
                costmap_2d::Costmap2DROS* costmap_ros_;
                double step_size_, min_dist_from_robot_;
                costmap_2d::Costmap2D* costmap_;
                base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
		ros::Time tg;
		unsigned int old_size;
		tf::TransformListener * handed_tf;
                ros::Subscriber odom_subscriber_;
                ros::Publisher pose_publisher_;
                ros::Publisher test_publisher_;
                std_msgs::Int32 test_pub_val_;
                geometry_msgs::PoseStamped desired_triad_pose_;
                std::vector< geometry_msgs::PoseStamped > g_plan_;
                ros::NodeHandle nh_;
                double controller_rate_, controller_dt_;
                double acc_lim_theta_,acc_lim_x_,max_vel_theta_,max_vel_x_;
                double min_vel_theta_,min_vel_x_;
                double max_reorientation_omega_;
                int ipose_,nposes_;
                tf::StampedTransform stfBaseLinkWrtOdom_; //base link w/rt odom frame; get this from tf; 
                tf::StampedTransform stfOdomWrtMap_;
                tf::StampedTransform stfEstBaseWrtMap_; 
                tf::TransformListener * tf_;
                nav_msgs::Odometry current_odom_;
                geometry_msgs::Pose odom_pose_;
                double odom_x_,odom_y_;
                double odom_phi_;
                double x_base_wrt_map_,y_base_wrt_map_,phi_base_wrt_map_, projected_travel_dist_;
                double path_heading_,heading_err_,lateral_err_, travel_dist_err_;
                double current_omega_cmd_,current_vx_cmd_,des_state_vel_;
                double omega_tol_,dist_tol_,heading_tol_;
                double min_theta_brake_dist_,min_lin_brake_dist_;
                double des_decel_,des_lin_brake_dist_;
                double K_omega_,K_lateral_,K_travel_dist_;
                double nom_travel_speed_;
                double tx_,ty_,nx_,ny_;
                double des_state_x_,des_state_y_;
                double old_subgoal_x_,old_subgoal_y_,new_subgoal_x_,new_subgoal_y_;
                double subgoal_dx_,subgoal_dy_,subgoal_dist_,des_progress_to_subgoal_;
                double dx_from_prior_subgoal_,dy_from_prior_subgoal_;
                double path_lookahead_dist_;
                geometry_msgs::Quaternion odom_quat_; 
                bool got_odom_;
                geometry_msgs::PoseStamped odom_poseStamped_wrt_map_;
                bool rotating_to_start_,done_with_path_;
	};	
};
#endif
