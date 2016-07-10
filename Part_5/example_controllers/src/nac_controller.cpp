//an NAC controller using emulated F/T sensor for feedback
// command nominal attractor at z = 0.5m, w/ K_virt and B_virt and M_virt

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace std;

const double  M_virt = 1.0; //1Kg
const double  K_virt = 1000.0; // N/m virtual stiffness
const double  B_virt = 50.0; // N/(m/sec) virtual damping
const double v_ideal_sat = 1.0; // put a cap on max vel cmd
const double  x_attractor = -0.2; //nominal position at zero force
//global vars for link state
double g_link2_pos=0.0;
double g_link2_vel= 0.0; 
//global var for ft sensor
double g_force_z = 0.0;

//a simple saturation function; provide saturation threshold, sat_val, and arg to be saturated, val
double sat(double val, double sat_val) {
    if (val>sat_val)
        return (sat_val);
    if (val< -sat_val)
        return (-sat_val);
    return val;
    
}

//subscribe to robot state on topic one_DOF_robot/joint_states w/ type sensor_msgs/JointState, fields position[0] and velocity[0]
//
void jnt_state_CB(const sensor_msgs::JointState& js) 
{ 
  g_link2_pos = js.position[0];
  g_link2_vel = js.velocity[0];
} 

//subscribe to ft_sensor_topic w/ type geometry_msgs/WrenchStamped;
// extract component wrench.force.z
void ft_sensor_CB(const geometry_msgs::WrenchStamped& ft) 
{ 
  g_force_z = ft.wrench.force.z;
} 
   
int main(int argc, char **argv) {
    ros::init(argc, argv, "nac_controller"); 
    ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
    ros::Publisher cmd_publisher = nh.advertise<std_msgs::Float64>("/one_DOF_robot/joint1_velocity_controller/command", 1);
    
    std_msgs::Float64 v_cmd_float64; //create a variable of type "Float64", 
    
   //need to sense robot state and f/t sensor    
    ros::Subscriber jnt_state_subscriber = nh.subscribe("one_DOF_robot/joint_states",1,jnt_state_CB);
    ros::Subscriber ft_sensor_subscriber = nh.subscribe("ft_sensor_topic",1,ft_sensor_CB);

    //input_float.data = 0.0;
   double v_cmd=0.0;
   double x_cmd=0.0;
   double x_amp=0.0;
   double freq,omega;
   double acc_ideal = 0.0;
   double v_ideal = 0.0;
   //cout<<"enter displacement amplitude: ";
   //cin>>x_amp;
   //cout<<"enter freq (in Hz): ";
   //cin>>freq;
   //omega = freq*2.0*M_PI;
   //double phase=0;
   double dt = 0.001;
   ros::Rate sample_rate(1/dt); 
   double f_net= 0.0;
   double f_virt = 0.0;

   
    while (ros::ok()) 
    {
        ros::spinOnce();  
        f_virt = K_virt*(x_attractor-g_link2_pos) + B_virt*(0-g_link2_vel);
        f_net =    g_force_z + f_virt;
        acc_ideal = f_net/M_virt; 
        v_ideal+= acc_ideal*dt;
        v_ideal = sat(v_ideal, v_ideal_sat);
        v_cmd_float64.data = v_ideal;

        cmd_publisher.publish(v_cmd_float64); // publish the value--of type Float64-- 
        ros::spinOnce();
	sample_rate.sleep(); 
    }
}

