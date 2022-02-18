/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }

enum struct SP_mode
{
	kPos = 0,
	kVel = 1
};

class Drone
{
private:
  ros::Subscriber state_sub;
	ros::Subscriber sp_pos_sub;
	ros::Subscriber sp_vel_sub;
	ros::Subscriber odom_sub;
  ros::Publisher local_pos_pub;
	ros::Publisher local_vel_pub;
	ros::Publisher global_pose_pub_;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

	std::string ns;

  mavros_msgs::State current_state;

	geometry_msgs::Pose sp_pose, current_pose;
	geometry_msgs::Twist sp_vel;
	SP_mode sp_mode;

	Eigen::Vector3d takeoff_pose;
	Eigen::Vector3d current_pose_vec;
	Eigen::Vector3d offset;
	bool pci_ready;  // Check if the sp are to be followed
public:
  Drone(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void odom_cb(const geometry_msgs::PoseStamped& odom);
	void sp_vel_cb(const geometry_msgs::Twist& vel_sp);
	void sp_pos_cb(const geometry_msgs::Pose& pos_sp);
	void timer_cb(const ros::TimerEvent&);
	
	bool initialize_drone();
	void initialize_variables();
	void initialize_pub_sub();
  void start();
	bool set_offboard();
	bool arm_drone();
	void takeoff();
	bool target_reached(double , Eigen::Vector3d);
	void controller();


};



// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "offb_node");
//     ros::NodeHandle nh;

//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//             ("mavros/state", 10, state_cb);
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//             ("mavros/setpoint_position/local", 10);
//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//             ("mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//             ("mavros/set_mode");

//     //the setpoint publishing rate MUST be faster than 2Hz
//     ros::Rate rate(20.0);

//     // wait for FCU connection
//     while(ros::ok() && !current_state.connected){
//         ros::spinOnce();
//         rate.sleep();
//     }

//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 2;

//     //send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         local_pos_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     ros::Time last_request = ros::Time::now();

//     while(ros::ok()){
//         if( current_state.mode != "OFFBOARD" &&
//             (ros::Time::now() - last_request > ros::Duration(5.0))){
//             if( set_mode_client.call(offb_set_mode) &&
//                 offb_set_mode.response.mode_sent){
//                 ROS_INFO("Offboard enabled");
//             }
//             last_request = ros::Time::now();
//         } else {
//             if( !current_state.armed &&
//                 (ros::Time::now() - last_request > ros::Duration(5.0))){
//                 if( arming_client.call(arm_cmd) &&
//                     arm_cmd.response.success){
//                     ROS_INFO("Vehicle armed");
//                 }
//                 last_request = ros::Time::now();
//             }
//         }

//         local_pos_pub.publish(pose);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }